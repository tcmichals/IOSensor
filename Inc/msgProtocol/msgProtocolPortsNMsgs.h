#pragma once

#include <cstdint>
#include <string>
#include <sstream>

namespace msgProtocol
{

enum class msgPorts:uint16_t
{

	ping,
	pwmInServer,  //consumer of pwmInClient
	pwmInClient,
	LEDClient,
	LEDServer,    //consumer if LEDClient
	dshotClient,
	dshotServer,  //consumer if dshotClient
	logMsg,
	pingTransport, //transport to transport

	dynamicPort = 0x8000,
};


typedef struct
{
/* 0 */	uint16_t m_version;
/* 2 */	uint16_t m_toPort;
/* 4 */	uint16_t m_fromPort;
/* 6 */	uint16_t m_route;
/* 8 */	uint16_t m_arg0;
/* 10*/ uint16_t m_arg1;
/* 12*/ uint16_t m_arg2;
/* 14*/ uint16_t m_len;
/* 16*/ uint8_t m_msg[];

/* 16 byte header */
}msgHdr_t;


inline
std::string msgPortToString(const msgPorts port)
{
	std::stringstream _str;
	switch(port)
	{
	case msgPorts::ping:
		_str << "msgPorts::ping";
		break;
	case msgPorts::pwmInServer:
		_str << "msgPorts::pwmInServer";
		break;
	case msgPorts::pwmInClient:
		_str << "msgPorts::pwmInClient";
		break;
	case msgPorts::LEDClient:
		_str << "msgPorts::LEDClient";
		break;
	case msgPorts::LEDServer:
		_str << "msgPorts::LEDServer";
		break;
	case msgPorts::dshotClient:
		_str << "msgPorts::dshotClient";
		break;
	case msgPorts::dshotServer:
		_str << "msgPorts::dshotServer";
		break;
	case msgPorts::logMsg:
		_str << "msgPorts::logMsg";
		break;

	default:
		_str << "msgPorts::logMsg";
		break;
	}
	return _str.str();
}
inline
std::string msgHdrToString(const msgHdr_t &msg)
{
	std::stringstream _str;

	_str << "m_version:" << msg.m_version;
	_str << " m_toPort:" << msgPortToString(static_cast<msgPorts>(msg.m_toPort));
	_str << " m_fromPort:" << msgPortToString(static_cast<msgPorts>(msg.m_fromPort));

	return _str.str();
}


/** Log Msg ***/
typedef struct
{
    uint16_t version;       //version 0
    uint16_t m_msgType;
    uint16_t m_arg;
    uint16_t m_lenOfMsg;
    char 	m_msg[];
}logMsg_t;



/** LED  **/

enum class ledMsg: uint16_t
{
	ledUpdateMsg,

};



typedef struct
{
	uint16_t 	m_version;       //version 0
	ledMsg 	 	m_msgType;
	uint16_t 	m_arg;
	uint16_t 	m_lenOfMsg;
}ledMsgHdr_t;


typedef struct
{
	uint8_t m_led;
    uint8_t m_red;
    uint8_t m_green;
    uint8_t m_blue;
}ledSingleLED_t;


/*** PWMIn  *****/

enum class pwmInMsg: uint16_t
{
	pwmInChannelsMsg,

};

typedef struct
{
	uint16_t 	m_version;       //version 0
	pwmInMsg	m_msgType;
	uint16_t 	m_arg;
	uint16_t 	m_lenOfMsg;

}pwmInMsgHdr_t;

typedef struct
{
    uint16_t counter;
    uint16_t numOfChannels;
 //   uint16_t channels[];
}pwmInChannelMsg_t;



}




