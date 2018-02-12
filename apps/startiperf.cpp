

#include "msgLogger.h"
#include "startUp.h"

#include <cstdint>
#include <cstring>
#include <cstdlib>

// server_start.cpp - start iperf
//  called from command line
#include "lwip/udp.h"
#include "lwip/tcpip.h"
#include "lwip/apps/lwiperf.h"

#include "msgProtocolPortsNMsgs.h"
#include "msgProtocol.h"




#define LWIP_REPORT_MSG 512
static char lwipReportMsg[LWIP_REPORT_MSG + sizeof(msgProtocol::logMsg_t)];

void iperfReport(void *arg, 
                 enum lwiperf_report_type report_type,
                const ip_addr_t* local_addr, 
                 u16_t local_port, 
                 const ip_addr_t* remote_addr, 
                 u16_t remote_port,
                u32_t bytes_transferred, 
                 u32_t ms_duration, 
                 u32_t bandwidth_kbitpsec)

{
    (void)arg;
    (void)local_addr;
    (void)local_port;
    (void)remote_addr;
    (void)remote_port;
    
    size_t lenOfMsg = 0;
    memset(lwipReportMsg, 0, sizeof(lwipReportMsg));
    char *pMsg =  &lwipReportMsg[sizeof(msgProtocol::logMsg_t)];
    msgProtocol::logMsg_t *pLogMsgHdr = reinterpret_cast<msgProtocol::logMsg_t*>(lwipReportMsg);
    
    switch(report_type)
    {

        case LWIPERF_TCP_DONE_SERVER:
            lenOfMsg = sprintf(pMsg, "Server: bytes_transferred(%" PRIu32 ") ms(%" PRIu32 ") kbps(%" PRIu32 ")", bytes_transferred, ms_duration, bandwidth_kbitpsec);
            break;

        case LWIPERF_TCP_DONE_CLIENT:
            lenOfMsg = sprintf(pMsg, "Client: bytes_transferred(%" PRIu32 ") ms(%" PRIu32 ") kbps(%" PRIu32 ")", bytes_transferred, ms_duration, bandwidth_kbitpsec);
            break;

        case LWIPERF_TCP_ABORTED_LOCAL:
            lenOfMsg = sprintf(pMsg, "LWIPERF_TCP_ABORTED_LOCAL");
            break;

        case LWIPERF_TCP_ABORTED_LOCAL_DATAERROR:
             lenOfMsg = sprintf(pMsg, "LWIPERF_TCP_ABORTED_LOCAL_DATAERROR");
            break;

        case LWIPERF_TCP_ABORTED_LOCAL_TXERROR:
             lenOfMsg = sprintf(pMsg, "LWIPERF_TCP_ABORTED_LOCAL_TXERROR");
            break;

        case LWIPERF_TCP_ABORTED_REMOTE:
             lenOfMsg = sprintf(pMsg, "LWIPERF_TCP_ABORTED_REMOTE");
            break;
    }
    
    
    pLogMsgHdr->m_lenOfMsg = lenOfMsg;

    postMsg(msgProtocol::msgPorts::logMsg,
    		msgProtocol::msgPorts::dynamicPort,
			static_cast<void *>(pLogMsgHdr),
			lenOfMsg + sizeof(*pLogMsgHdr));

}

static void _startUplwIPIPerf(void) __attribute__((constructor(102)));
void _startUplwIPIPerf(void)
{
      regStartUpFunction(StartUpPriority::_6,
                       "lwIPIPerf",
                       (startUpFunctor_t)[]()->bool
                       {
                               if(ERR_OK != tcpip_callback_with_block([](void* ctx) -> void
                                           {
                                             lwiperf_start_tcp_server_default(iperfReport, nullptr);
                                           },
                                           nullptr,
                                           0))
                            return true;
                              return false;
                        });  
    
}
