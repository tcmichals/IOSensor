
#include <cstdint>
#include <array>
#include <memory>

#include <map>
#include <vector>

#include "msgProtocol/msgProtocolPortsNMsgs.h"
#include "msgProtocol/msgProtocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"



namespace msgProtocol
{

static  SemaphoreHandle_t m_mutex;

static std::map<msgPorts, postMsg_CallbackFP_t> _gMsgRouter;


registerMsgPort_Rt_t registerMsgPort(msgPorts port,
									const postMsg_CallbackFP_t &msgEvtCallback)
{

	xSemaphoreTake(m_mutex,portMAX_DELAY);

	if(_gMsgRouter.end() == _gMsgRouter.find(port))
	{
		_gMsgRouter[port] = msgEvtCallback;
		xSemaphoreGive(m_mutex);
		return registerMsgPort_Rt_t::OK;
	}

	xSemaphoreGive(m_mutex);
	return registerMsgPort_Rt_t::ALREADY_USED;
}

registerMsgPort_Rt_t registerMsgPort(const std::vector<msgPorts> &portVector,
									const postMsg_CallbackFP_t &msgEvtCallback)
{
	for(auto &port: portVector)
		if ( registerMsgPort_Rt_t::ALREADY_USED == registerMsgPort(port, msgEvtCallback))
				return registerMsgPort_Rt_t::ALREADY_USED;

	return registerMsgPort_Rt_t::OK;

}

postMsgPort_Rt_t postMsg(msgPorts toPort,
		 	 	 	 	 msgPorts fromPort,
                        void     *pMsg,
                         uint16_t  lenOfMsg)
{
	postMsgPort_Rt_t rc = postMsgPort_Rt_t::OK;
	postMsg_CallbackFP_t _callback = nullptr;

	{
		xSemaphoreTake(m_mutex,portMAX_DELAY);
		if(_gMsgRouter.end() != _gMsgRouter.find(toPort))
		{
			_callback = _gMsgRouter[toPort];
		}
		xSemaphoreGive(m_mutex);
	}

	if (_callback)
		rc = _callback(toPort, fromPort, pMsg, lenOfMsg);

	return rc;
}

static void _startUpMsgProtocol(void) __attribute__((constructor(102)));
void _startUpMsgProtocol(void)
{
	m_mutex = xSemaphoreCreateMutex();
}

}


//eof
