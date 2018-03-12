
#define LOGMSGAPI

#include <array>
#include <atomic>
#include <memory>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

//#include "timestamp.pb.h"
#include "messages.pb.h"
#include "ringBuffer.h"
#include "startUp.h"
#include "udpServer.h"
#include "msgLogger.h"

#include "string.h"
#include <stdarg.h>
#include <stdio.h>

#include "lwip/debug.h"
#include "lwip/opt.h"
#include "lwip/stats.h"
#include "lwip/tcpip.h"
#include "lwip/udp.h"
//nano pb..
#include <pb_encode.h>
#include <pb_decode.h>

#define LC_INCLUDE "lc-addrlabels.h"
#include "pt.h"

#define NUM_MSGS 10
#define STACK_SIZE_PHANDLER ((sizeof(UdpMessage) * 4) + 512)
#define PRIORITY_PHANDLER (tskIDLE_PRIORITY + 2)

static StaticTask_t gUDPServicesTskHandleStatic;
static TaskHandle_t gUDPServicesTskHandle;
static std::array<StackType_t, STACK_SIZE_PHANDLER> stackPHandlerUDPServer;

#define UDP_PORT_IO 56000

class udpServer
{

  public:
    enum class event_t : uint32_t
    {
        startOf_Cmds,

        startUDPSocket_Cmd,
        sendUDPPkt_Cmd,

        endOf_Cmds,

        startOfEvents,

        lwIPUDPSocketEvent,

        endOfEvents,

    };

    typedef struct
    {
        event_t     m_event;
        udpServer*  m_this;
        UdpMessage  m_msg;
    } lwipArgs_t;

  protected:
      
    struct udp_pcb*     udpServer_pcb;
    struct pt           m_cmdThread;
    struct pt           m_udpSocketThread;
    
    SemaphoreHandle_t   m_mutex;
    StaticSemaphore_t   xMutexBuffer;
    
    QueueHandle_t       m_msgQueueHandle;
    StaticQueue_t       m_msgStaticQueue;
    
    
    ip_addr_t           m_ipLastAddr;
    u16_t               m_pastPort;
    
    std::array<lwipArgs_t, NUM_MSGS>    m_msgArray;
    ringBuffer<lwipArgs_t*, NUM_MSGS>   m_msgRingBuffer;
    
    // MAX SIZE OF UDP PACKET.. 
    std::array<uint8_t, 1500> m_inputBuffer;
    std::array<uint8_t, sizeof(UdpMessage) *2> m_outputBuffer;
    std::array<uint8_t, sizeof(UdpMessage) *5> m_msgQueueStorage;
    
    bool m_transportActive;

  protected:
    lwipArgs_t* allocArgs();
    void freeArgs(lwipArgs_t* arg);
    int cmdThread(struct pt*, lwipArgs_t* args);
    int socketThread(struct pt*, struct pbuf* pBuf);
    bool routeMsg(const UdpMessage &msg );
    void createSocket();

    const char* decodeEvt(event_t evt)
    {
        switch(evt)
        {

        case event_t::startOf_Cmds:
            return "startOf_Cmds";
            break;
        case event_t::startUDPSocket_Cmd:
            return "startUDPSocket_Cmd";
            break;
        case event_t::sendUDPPkt_Cmd:
            return "sendUDPPkt_Cmd";
            break;
        case event_t::endOf_Cmds:
            return "endOf_Cmds";
            break;
        case event_t::startOfEvents:
            return "startOfEvents";
            break;
        case event_t::lwIPUDPSocketEvent:
            return "lwIPUDPSocketEvent";
            break;
        case event_t::endOfEvents:
            return "endOfEvents";
            break;
        default:
            return "unknown";
        }
    }

  public:
    udpServer();
    virtual ~udpServer();

    void freeRTOSThread(void* arg);
    bool sendCmdTolwIP(const lwipArgs_t &msg );
};

inline udpServer::udpServer(): udpServer_pcb(nullptr)
{
    memset(&m_ipLastAddr, 0, sizeof(m_ipLastAddr));
}

inline udpServer::~udpServer()
{
}

inline void udpServer::createSocket()
{
    udpServer_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);

    if(udpServer_pcb != nullptr)
    {
        err_t err;
        err = udp_bind(udpServer_pcb, IP_ANY_TYPE, UDP_PORT_IO);
        if(err == ERR_OK)
        {
            SYSLOGMSG(LogMsg_Debug, "%s:%d:evt=%s", __PRETTY_FUNCTION__, __LINE__, "UDP Socket open");
            udp_recv(udpServer_pcb,
                     [](void* arg, struct udp_pcb* upcb, struct pbuf* p, const ip_addr_t* addr, u16_t port) {
                         (void)upcb;
                         udpServer* pServer = reinterpret_cast<udpServer*>(arg);

                         if(pServer)
                         {
                             pServer->m_ipLastAddr = *addr;
                             pServer->m_pastPort = port;
                             pServer->socketThread(&pServer->m_udpSocketThread, p);
                         }
                         pServer->m_transportActive = true;
                         //    SYSLOGMSG(LogMsg_Debug, "%s:%d:+", __PRETTY_FUNCTION__, __LINE__);
                     },
                     this);
        }
        else
        {
            /* abort? output diagnostic? */
        }
    }
    else
    {
        /* abort? output diagnostic? */
    }
}

inline int udpServer::cmdThread(struct pt* pt, lwipArgs_t* args)
{
    PT_BEGIN(pt);

    while(true)
    {
        PT_WAIT_WHILE(pt, !((args->m_event > event_t::startOf_Cmds) && (args->m_event < event_t::endOf_Cmds)));
        /* OK start UDP Socket */
        // SYSLOGMSG(LogMsg_Debug, "%s:%d:evt=%s", __PRETTY_FUNCTION__, __LINE__, decodeEvt(args->m_event));

        switch(args->m_event)
        {
        case event_t::startUDPSocket_Cmd:
        {
            createSocket();
        }
        break;

        case event_t::sendUDPPkt_Cmd:
        {
            if(m_ipLastAddr.addr && udpServer_pcb)
            {
                //OK need to serialize message .. 
                // SYSLOGMSG(LogMsg_Debug, "%s:%d: UDP sending pkt", __PRETTY_FUNCTION__, __LINE__);
                
                //encode message
                pb_ostream_t stream = pb_ostream_from_buffer(m_outputBuffer.data(), m_outputBuffer.size());
                if (pb_encode_delimited(&stream, UdpMessage_fields, &args->m_msg))
                {
                    //create a pbuff to send via UDP
                    struct pbuf* pbufMsg = pbuf_alloc(PBUF_TRANSPORT, stream.bytes_written, PBUF_RAM);
                    
                    //copy user buffer into UDP
                    if (!pbufMsg || (ERR_OK != pbuf_take(pbufMsg, m_outputBuffer.data(), stream.bytes_written)))
                    {
                        if(ERR_OK == udp_sendto(udpServer_pcb, pbufMsg, &m_ipLastAddr, UDP_PORT_IO))
                            m_transportActive = true;
                        
                        pbuf_free(pbufMsg);  //free pbuff
                    }
                }
            }
            else
            {
                SYSLOGMSG(LogMsg_Debug, "%s:%d: UDP not connected", __PRETTY_FUNCTION__, __LINE__);
            }
        }
        break;

        default:
            break;
        }
        PT_YIELD(pt);
    }

    PT_END(pt);
}

inline int udpServer::socketThread(struct pt* pt, struct pbuf* pBuf)
{
    PT_BEGIN(pt);

    while(true)
    {
        SYSLOGMSG(LogMsg_Debug, "%s:%d: msg size:pBuf->tot_len=%d", __PRETTY_FUNCTION__, __LINE__, pBuf->tot_len);
        if(pBuf)
        {

            u16_t lenCopied = pbuf_copy_partial(pBuf, m_inputBuffer.data(),
                                                std::min(m_inputBuffer.size(), (size_t)pBuf->tot_len), 0);
                
            pb_istream_t input = pb_istream_from_buffer(m_inputBuffer.data(), lenCopied);
                
            UdpMessage request= {};
            if (false == pb_decode_delimited(&input, UdpMessage_fields, &request))
            {
                SYSLOGMSG(LogMsg_Debug, "%s:%d: DECODE FAILED", __PRETTY_FUNCTION__, __LINE__);
                //free packet
                pbuf_free(pBuf);
                break;
            }
            
            if ( xQueueSend(m_msgQueueHandle , &request, 0))
            {
                SYSLOGMSG(LogMsg_Debug, "%s:%d: POST FAILED", __PRETTY_FUNCTION__, __LINE__);
            }
            //free packet.. 
            pbuf_free(pBuf);

        }

        PT_YIELD(pt);
        // OK it is an UDP event
    }

    PT_END(pt);
}

inline udpServer::lwipArgs_t* udpServer::allocArgs()
{
    lwipArgs_t* rc = nullptr;

    if(xSemaphoreTake(m_mutex, portMAX_DELAY))
    {
        m_msgRingBuffer.pop(rc);
    }

    xSemaphoreGive(m_mutex);

    return rc;
}

inline void udpServer::freeArgs(lwipArgs_t* arg)
{
    if(arg && xSemaphoreTake(m_mutex, portMAX_DELAY))
    {
        m_msgRingBuffer.push(arg);
        xSemaphoreGive(m_mutex);
    }
}

inline 
bool udpServer::sendCmdTolwIP(const lwipArgs_t &msg)
{
    //  SYSLOGMSG(LogMsg_Debug, "%s:%d:+", __PRETTY_FUNCTION__, __LINE__);
    // Going to post a message to udpServer thread in lwIP
    lwipArgs_t* pMsg = allocArgs();
    if( !pMsg )
    {
        SYSLOGMSG(LogMsg_Debug, "%s:%d:+", __PRETTY_FUNCTION__, __LINE__);
        return false;
    }
    // copy message
    *pMsg = msg;

    if(ERR_OK != tcpip_callback_with_block([](void* ctx) -> void 
                     {
                         lwipArgs_t* pMsg = reinterpret_cast<lwipArgs_t *>(ctx);
                         if( pMsg )
                         {
                             pMsg->m_this->cmdThread(&pMsg->m_this->m_cmdThread, pMsg);
                             pMsg->m_this->freeArgs(pMsg);
                         }
                     },
                     reinterpret_cast<void*>(pMsg), 0))
    {
        if(pMsg)
        {
            freeArgs(pMsg);
        }
        return false;
    }

    //  SYSLOGMSG(LogMsg_Debug, "%s:%d:-", __PRETTY_FUNCTION__, __LINE__);
    return true;
}


bool udpServer::routeMsg(const  UdpMessage &msg ) 
{
    lwipArgs_t args;
    
    memset(&args, 0, sizeof(args));
    
    args.m_event = event_t::sendUDPPkt_Cmd;
    args.m_this = this;
    args.m_msg = msg;
    
    // OK need to route a msg out to UDP...
    return sendCmdTolwIP(args);
}

     


void udpServer::freeRTOSThread(void* arg)
{

    (void)arg;
    PT_INIT(&m_cmdThread);
    PT_INIT(&m_udpSocketThread);

    m_mutex = xSemaphoreCreateMutexStatic(&xMutexBuffer);
    m_msgQueueHandle = xQueueCreateStatic(NUM_MSGS, 
                                          sizeof(UdpMessage),
                                          m_msgQueueStorage.data(),
                                          &m_msgStaticQueue);
                                          
                                          

    for(size_t _index = 0; _index < m_msgArray.size(); _index++)
    {
        m_msgRingBuffer.push(&m_msgArray[_index]);
    }

    lwipArgs_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.m_event = event_t::startUDPSocket_Cmd;
    sendCmdTolwIP(msg);

    while(1)
    {
        TickType_t ticks = 500 / portTICK_PERIOD_MS;
        UdpMessage msg;
        BaseType_t  rc = xQueueReceive(m_msgQueueHandle,
                                        &msg,
                                        ticks);
         if (pdTRUE == rc)
         {
             //Received a message over UDP.. route.. or send
             switch ( msg.which_message)
             {
                 case UdpMessage_pingReq_tag:
                 {
                     //turn around the message .. 
                     UdpMessage pingMsg;
                     
                     memset(&pingMsg, 0, sizeof(pingMsg));
                     pingMsg.id = msg.id;
                     pingMsg.type = MessageType_Response;
                     pingMsg.message.pingReq.foo = msg.message.pingReq.foo;
                     pingMsg.which_message = UdpMessage_pingResp_tag;
                     //send .. 
                     routeMsg(pingMsg);
                 }
                 break;
              
             }
         }
                                        
        

        m_transportActive = false;
    }
}

static __attribute__((section(".ccmram"))) udpServer gUDPServer;

static void _startUpUdpServices(void) __attribute__((constructor(102)));
void _startUpUdpServices(void)
{

    regStartUpFunction(StartUpPriority::_6, "udpServices", (startUpFunctor_t)[]()->bool {
        gUDPServicesTskHandle =
            xTaskCreateStatic([](void* args) -> void { gUDPServer.freeRTOSThread(args); }, "udpServices",
                              STACK_SIZE_PHANDLER, (void*)&gUDPServer, PRIORITY_PHANDLER,
                              (StackType_t*)stackPHandlerUDPServer.data(), &gUDPServicesTskHandleStatic);

        if(gUDPServicesTskHandle)
        {
            return true;
        }
        return false;
    });
}

// eof
