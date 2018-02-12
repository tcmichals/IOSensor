
#define LOGMSGAPI

#include <array>
#include <memory>
#include <atomic>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ringBuffer.h"
#include "msgLogger.h"
#include "startUp.h"
#include "udpServer.h"

#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"

#define LC_INCLUDE "lc-addrlabels.h"
#include "pt.h"

#define NUM_MSGS 10
#define STACK_SIZE_PHANDLER 512
#define PRIORITY_PHANDLER (tskIDLE_PRIORITY + 2)

static StaticTask_t gUDPServicesTskHandleStatic;
static TaskHandle_t gUDPServicesTskHandle;
static std::array<StackType_t, STACK_SIZE_PHANDLER> stackPHandlerUDPServer;

#define MAX_MSG_SIZE 512
#define MAX_MSG_QUEUE 5
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
    	event_t 				m_event;
    	struct pbuf*  			m_pbufMsg; //lwipBuffer
    	uint32_t 				m_lenOfMsgToSend;
    	udpServer   			*m_this;
    } lwipArgs_t;

protected:
    struct udp_pcb* udpServer_pcb;
    struct pt m_cmdThread;
    struct pt m_udpSocketThread;
    SemaphoreHandle_t m_mutex;
    StaticSemaphore_t xMutexBuffer;
    ip_addr_t   m_ipLastAddr;
    u16_t       m_pastPort;
    std::array<lwipArgs_t, NUM_MSGS> m_argsArray;
    ringBuffer<lwipArgs_t*, NUM_MSGS> m_argsRingBuffer;
    std::array<uint8_t, MAX_MSG_SIZE> m_msgToRoute;

    bool m_transportActive;

protected:
    lwipArgs_t* allocArgs();
    void freeArgs(lwipArgs_t* arg);
    int cmdThread(struct pt*, lwipArgs_t* args);
    int socketThread(struct pt*, struct pbuf* pBuf);
    msgProtocol::postMsgPort_Rt_t  routeMsgs(msgProtocol::msgPorts toPort,
    										 msgProtocol::msgPorts fromPort,
											 void *pMsg,
											 uint16_t lenOfMsg);
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
    bool sendCmdTolwIP(msgProtocol::msgPorts toPort,
    									 msgProtocol::msgPorts fromPort,
    									 void     *pMsg,
    									 uint16_t  lenOfMsg,
    									 event_t evt);
};

inline udpServer::udpServer()
    : udpServer_pcb(nullptr)

{
    memset(&m_ipLastAddr, 0, sizeof(m_ipLastAddr));
}

inline udpServer::~udpServer()
{

}

inline
void udpServer::createSocket()
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
                     [](void* arg,
                        struct udp_pcb* upcb,
                        struct pbuf* p,
                        const ip_addr_t* addr,
                        u16_t port)
                     {
            			(void)upcb;
                         udpServer* pServer = reinterpret_cast<udpServer*>(arg);

                         if (pServer)
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

inline
int udpServer::cmdThread(struct pt* pt, lwipArgs_t* args)
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
            if(m_ipLastAddr.addr && args->m_pbufMsg && udpServer_pcb)
            {
            	// SYSLOGMSG(LogMsg_Debug, "%s:%d: UDP sending pkt", __PRETTY_FUNCTION__, __LINE__);
               if ( ERR_OK== udp_sendto(udpServer_pcb, args->m_pbufMsg, &m_ipLastAddr, UDP_PORT_IO))
            	   m_transportActive = true;

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
        	//need a packet reassembly UDP can fragment the packet also..
        	if (pBuf->tot_len >= sizeof( msgProtocol::msgHdr_t))
        	{
        		u16_t lenCopied = pbuf_copy_partial(pBuf,
        											m_msgToRoute.data(),
													std::min(m_msgToRoute.size(),(size_t)pBuf->tot_len),
													0);
        		msgProtocol::msgHdr_t *pHdr = reinterpret_cast< msgProtocol::msgHdr_t*>( m_msgToRoute.data());

        		if ((lenCopied < m_msgToRoute.size()) &&
        				(pHdr->m_len >= (pBuf->tot_len - sizeof( msgProtocol::msgHdr_t))) )
        		{
        			//OK have a valid message route message..
        			msgProtocol::postMsg(static_cast<msgProtocol::msgPorts>(pHdr->m_toPort),
        								static_cast<msgProtocol::msgPorts>(pHdr->m_fromPort),
										static_cast<void *>(pHdr+1),
										pHdr->m_len);
        		}

        	}
        	// p->tot_len
        	//OK need to figure out
        	pbuf_free(pBuf);
            // OK got a packet
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
        m_argsRingBuffer.pop(rc);
    }

    xSemaphoreGive(m_mutex);

    return rc;
}

inline void udpServer::freeArgs(lwipArgs_t* arg)
{
    if(arg && xSemaphoreTake(m_mutex, portMAX_DELAY))
    {
        m_argsRingBuffer.push(arg);
        xSemaphoreGive(m_mutex);
    }
}

inline
bool udpServer::sendCmdTolwIP(msgProtocol::msgPorts toPort,
									 msgProtocol::msgPorts fromPort,
									 void     *pMsg,
									 uint16_t  lenOfMsg,
									 event_t evt)
{
  //  SYSLOGMSG(LogMsg_Debug, "%s:%d:+", __PRETTY_FUNCTION__, __LINE__);
    // Going to post a message to udpServer thread in lwIP
    lwipArgs_t* args = allocArgs();
    if(!args || ((lenOfMsg + sizeof(msgProtocol::msgHdr_t)) > MAX_MSG_SIZE))
    {
        SYSLOGMSG(LogMsg_Debug, "%s:%d:+", __PRETTY_FUNCTION__, __LINE__);
        return false;
    }

    struct pbuf* pbufMsg = nullptr;
    //setup hdr..
    msgProtocol::msgHdr_t msgHdr;
    memset(&msgHdr, 0, sizeof(msgHdr));
    msgHdr.m_toPort = static_cast<uint16_t>(toPort);
    msgHdr.m_fromPort = static_cast<uint16_t>(fromPort);
    msgHdr.m_len = lenOfMsg;

    memset(args,0, sizeof(*args));

    if(lenOfMsg)
    {
        pbufMsg = pbuf_alloc(PBUF_TRANSPORT, lenOfMsg + sizeof(msgProtocol::msgHdr_t), PBUF_RAM);
        if(!pbufMsg ||
        		(ERR_OK != pbuf_take_at(pbufMsg, pMsg, lenOfMsg, sizeof(msgProtocol::msgHdr_t))) ||
        		(ERR_OK != pbuf_take(pbufMsg, &msgHdr, sizeof(msgHdr))))
        {
           SYSLOGMSG(LogMsg_Debug, "%s:%d:ERROR-", __PRETTY_FUNCTION__, __LINE__);
           freeArgs(args);
           return false;
        }
        args->m_lenOfMsgToSend = lenOfMsg + sizeof(msgProtocol::msgHdr_t);
     }

    // need to post to lwIP thread..
    args->m_this = this;
    args->m_event = evt;
    args->m_pbufMsg =pbufMsg;

    if(ERR_OK != tcpip_callback_with_block([](void* ctx) -> void
                                           {
                                               udpServer::lwipArgs_t* args = reinterpret_cast<udpServer::lwipArgs_t*>(ctx);
                                               if (args && args->m_this)
                                               {
                                            	   args->m_this->cmdThread(&args->m_this->m_cmdThread, args);
                                                   if(args)
                                                   {
                                                	   pbuf_free(args->m_pbufMsg);
                                                	   args->m_this->freeArgs(args);
                                                    }
                                                }
                                           },
                                           reinterpret_cast<void*>(args),
                                           0))
    {
        if(args->m_pbufMsg)
        {
            pbuf_free(args->m_pbufMsg);
        }
        
        if (args)
        {
            freeArgs(args);
        }
        return false;
    }
    
  //  SYSLOGMSG(LogMsg_Debug, "%s:%d:-", __PRETTY_FUNCTION__, __LINE__);
    return true;
}


msgProtocol::postMsgPort_Rt_t  udpServer::routeMsgs(msgProtocol::msgPorts toPort,
										 msgProtocol::msgPorts fromPort,
										 void *pMsg,
										 uint16_t lenOfMsg)
{
	//OK need to route a msg out to UDP...

	sendCmdTolwIP(toPort, fromPort, pMsg, lenOfMsg, event_t::sendUDPPkt_Cmd);

	return msgProtocol::postMsgPort_Rt_t::OK;
}
extern bool isUSBLinkActive(void);
extern bool isRadioLinkActive(void);

static const msgProtocol::ledSingleLED_t *getLEDState(bool on, bool isTrafficRx)
{

	static const msgProtocol::ledSingleLED_t m_ledSetting[4] =
	{
				 /*led red green blue */
				{ 	0, 	0, 		0,   0},  /* off */
				{	0, 	0, 		0,   0xFF},  /* radio link only */
				{	0, 	0xFF,	0, 	 0},  /* usb link only  */
				{	0, 	0, 	 0xFF,	 0},  /* both USB and radio */
	};

	if (on)
	{
		uint8_t state = 0;
		if (isUSBLinkActive() && isTrafficRx)
		{
			state|=(1<<1);
		}

        if (isRadioLinkActive())
        {
        	state|=1;
        }

		if (state < sizeof(m_ledSetting)/sizeof(msgProtocol::ledSingleLED_t ) )
		{
			return &m_ledSetting[state];
		}
	}

	return &m_ledSetting[0];

}

void udpServer::freeRTOSThread(void* arg)
{

	(void)arg;
    PT_INIT(&m_cmdThread);
    PT_INIT(&m_udpSocketThread);

    m_mutex = xSemaphoreCreateMutexStatic(&xMutexBuffer);

    for(size_t _index = 0; _index < m_argsArray.size(); _index++)
    {
        m_argsRingBuffer.push(&m_argsArray[_index]);
    }

	auto _callback = std::bind(& udpServer::routeMsgs,
								this,
								std::placeholders::_1,
								std::placeholders::_2,
								std::placeholders::_3,
								std::placeholders::_4);

	msgProtocol::registerMsgPort(msgProtocol::msgPorts::pwmInClient,_callback);

    sendCmdTolwIP(	msgProtocol::msgPorts::dynamicPort,
    				msgProtocol::msgPorts::dynamicPort,
					nullptr,
					0,
					event_t::startUDPSocket_Cmd);

    std::array<uint8_t, sizeof(msgProtocol::ledMsgHdr_t) + sizeof(msgProtocol::ledSingleLED_t)> msgLEDArray;
    msgProtocol::ledMsgHdr_t *pLEDHdr = reinterpret_cast<msgProtocol::ledMsgHdr_t *>(msgLEDArray.data());
    msgProtocol::ledSingleLED_t *pLEDMsg = reinterpret_cast<msgProtocol::ledSingleLED_t *>(pLEDHdr+1);

    memset(msgLEDArray.data(), 0, msgLEDArray.size());
    pLEDHdr->m_msgType = msgProtocol::ledMsg::ledUpdateMsg;
    pLEDHdr->m_lenOfMsg = sizeof(msgProtocol::ledSingleLED_t);

    pLEDMsg->m_led =7;
    bool blinkState = false;


    while(1)
    {
        TickType_t ticks = 500 / portTICK_PERIOD_MS;
        vTaskDelay(ticks);

        *pLEDMsg =  *getLEDState(blinkState, m_transportActive);
        pLEDMsg->m_led =7;
        if(blinkState)
        	blinkState = false;
        else
        	blinkState = true;


    	msgProtocol::postMsg(msgProtocol::msgPorts::LEDServer,
    						 msgProtocol::msgPorts::dynamicPort,
    						msgLEDArray.data(),
							msgLEDArray.size());

    	m_transportActive = false;

    }
}

static __attribute__((section(".ccmram"))) udpServer gUDPServer;

static void _startUpUdpServices(void) __attribute__((constructor(102)));
void _startUpUdpServices(void)
{

    regStartUpFunction(StartUpPriority::_6,
                       "udpServices",
                       (startUpFunctor_t)[]()->bool
                       {
                            gUDPServicesTskHandle = xTaskCreateStatic([](void* args) -> void
                                                  {
                                                      gUDPServer.freeRTOSThread(args);
                                                  },
                                                  "udpServices",
                                                  STACK_SIZE_PHANDLER,
                                                  (void*)&gUDPServer,
                                                  PRIORITY_PHANDLER,
                                                  (StackType_t*)stackPHandlerUDPServer.data(),
                                                  &gUDPServicesTskHandleStatic);

                            if(gUDPServicesTskHandle)
                            {
                                return true;
                            }
                            return false;
                    });
}

// eof
