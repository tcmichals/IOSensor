
#pragma once 
#include <stdint.h>


typedef enum
{
LogMsg_Debug,
LogMsg_Info,
logMsg_Warning,
logMsg_Error,
logMsg_None

}logMsgLevel_t;

#ifdef __cplusplus
#include <cstdint>
extern "C"
{
#else
    
#include <stdint.h>
#include <stdbool.h>    
    
#endif
   
   
    
    
bool setLogMsgLevel(logMsgLevel_t level);
logMsgLevel_t getLogMsgLevel();
bool logMsg(logMsgLevel_t level, const char *msg, ...);

#ifdef LOGMSGAPI
#define SYSLOGMSG(_level_,_msg_, ... ) logMsg(_level_, _msg_, __VA_ARGS__)
    
#else
#define SYSLOGMSG(_level_, _msg_, ...)
#endif
    
#ifdef __cplusplus
}
#endif
