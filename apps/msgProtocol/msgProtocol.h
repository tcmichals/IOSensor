


#pragma once
#include <cstdint>
#include <vector>
#include <functional>

namespace msgProtocol
{


enum class  postMsgPort_Rt_t
{
	OK,
    invalidToPort =0,
    invalidFromPort,
    msgToBig,
};


typedef std::function<postMsgPort_Rt_t
						( msgPorts toPort,
						  msgPorts fromPort,
						  void  *pMsg,
						 uint16_t lenOfMsg)>  postMsg_CallbackFP_t;

enum class  registerMsgPort_Rt_t
{
	OK,
    ALREADY_USED,
};

registerMsgPort_Rt_t registerMsgPort(msgPorts port,
									const postMsg_CallbackFP_t &msgEvtCallback);

registerMsgPort_Rt_t registerMsgPort(const std::vector<msgPorts> &portVector,
									const postMsg_CallbackFP_t &msgEvtCallback);


enum class msgRouterFlags_t
{
 None
};



 postMsgPort_Rt_t postMsg(msgPorts toPort,
		 	 	 	 	 msgPorts fromPort,
                         void     *pMsg,
                          uint16_t  lenOfMsg);


}
