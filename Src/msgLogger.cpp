#include <stdio.h>
#include <stdarg.h>
#include <cstring>
#include <array>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "msgLogger.h"
#include "ringBuffer.h"
#include "msgLogger.h"
#include "startUp.h"

#define LOG_MSG_SIZE 256
#define LOG_NUM_MSG 20

static logMsgLevel_t _gLogLevel;
static bool _glogInit;

typedef std::array<char, LOG_MSG_SIZE> logMsg_t;
static std::array<logMsg_t, LOG_NUM_MSG> _gLogMsgArray;
static ringBuffer<logMsg_t *, LOG_NUM_MSG> _gLogRingBuffer;

typedef struct
{
 logMsg_t *pLogMsg;
 int        lenOfMsg;
}msgMsg_t;

static std::array<msgMsg_t, LOG_NUM_MSG> _gmsgLogArray;

#define STACK_SIZE_PHANDLER 512
#define PRIORITY_PHANDLER  (tskIDLE_PRIORITY)

static StaticTask_t  gLogTaskHandleStatic;
static TaskHandle_t  gLogTskHandle;
static std::array<StackType_t, STACK_SIZE_PHANDLER> stackPHandlerLog;

static StaticQueue_t   _glogQueueStatic;
static QueueHandle_t  _glogQueue;


static bool logInit()
{
    for(size_t index =0; index < _gLogMsgArray.size(); index++)
        _gLogRingBuffer.push(&_gLogMsgArray[index]);
    
    _glogInit = true; 
    
    return _glogInit;
    
}
bool setLogMsgLevel(logMsgLevel_t level)
{
 
    _gLogLevel = level;
    return true;
    
}
logMsgLevel_t getLogMsgLevel()
{
    return _gLogLevel;
    
}


bool logMsg(logMsgLevel_t level, const char *format, ...)
{
  logMsg_t *pLogMsg = nullptr;
  
  if ( !_glogInit )
      return false;
  
  if ( level < _gLogLevel)
      return false;
  
   vTaskSuspendAll();
    bool rc =  _gLogRingBuffer.pop(pLogMsg);
   xTaskResumeAll();

   if (rc)
   {
        va_list aptr;
        va_start(aptr, format);
        int len = vsprintf(pLogMsg->data(), format, aptr);
        va_end(aptr);

        if ((*pLogMsg)[len-1] == '\n')
        {
            strcat(pLogMsg->data(), "\r");
            len+=1;
        }
        else
        {
            strcat(pLogMsg->data(), "\n\r");
            len+=2;
        }
        

        msgMsg_t msg;
        msg.pLogMsg = pLogMsg;
        msg.lenOfMsg = len;
      
        if (xQueueSendToBack(_glogQueue, reinterpret_cast<void *>(&msg), 0))
        {
            rc= true;
        }
        else
        {
             vTaskSuspendAll();
             _gLogRingBuffer.push(pLogMsg);
             xTaskResumeAll();
            rc = false;
        }
   }
   return rc;
}

extern "C" void txSerialPort(char *pStr, int len);
static void loggerTask(void *arg)
{
    (void)arg;


    while(true)
    {
        msgMsg_t msg;
        if (xQueueReceive(_glogQueue, &msg , 5000))
        {
         //   txSerialPort(msg.pLogMsg->data(), std::min(msg.lenOfMsg, LOG_MSG_SIZE));
              vTaskSuspendAll();
             _gLogRingBuffer.push(msg.pLogMsg);
              xTaskResumeAll();
        }

    }

}


static bool startUpLogger(void)
{
        _glogQueue = xQueueCreateStatic(LOG_NUM_MSG, 
                                    sizeof(msgMsg_t),
                                    (uint8_t *)_gmsgLogArray.data(),
                                    &_glogQueueStatic
                                   );
  
    setLogMsgLevel(LogMsg_Debug);
    logInit();
    
    logMsg(logMsg_Error,"\n\rStart of new log");
        
    gLogTskHandle = xTaskCreateStatic(loggerTask,
                                    "logger",
                                    STACK_SIZE_PHANDLER,
                                    nullptr,
                                    PRIORITY_PHANDLER  ,
                                    (StackType_t *)stackPHandlerLog.data(),
                                    &gLogTaskHandleStatic);
                                
                  
    if (gLogTskHandle)
    {
        return true;
    }
    else
    {
        return false;
    }
}


 extern "C" int lwiplogMsg(const char *format, ...)
 {
  logMsg_t *pLogMsg = nullptr;
  
  if ( !_glogInit )
      return false;
  
  
   vTaskSuspendAll();
    bool rc =  _gLogRingBuffer.pop(pLogMsg);
   xTaskResumeAll();

   if (rc)
   {
        va_list aptr;
        va_start(aptr, format);
        int len = vsprintf(pLogMsg->data(), format, aptr);
        va_end(aptr);

        if ((*pLogMsg)[len-1] == '\n')
        {
            strcat(pLogMsg->data(), "\r");
        
        }
      

        msgMsg_t msg;
        msg.pLogMsg = pLogMsg;
        msg.lenOfMsg = strlen(pLogMsg->data());
      
        if (xQueueSendToBack(_glogQueue, reinterpret_cast<void *>(&msg), 0))
        {
            rc= true;
        }
        else
        {
             vTaskSuspendAll();
             _gLogRingBuffer.push(pLogMsg);
             xTaskResumeAll();
            rc = false;
        }
   }
   return rc;
 }
 
 
static void _startUpLogger(void) __attribute__((constructor (102)));
void _startUpLogger(void) 
{
    regStartUpFunction(StartUpPriority::Highest, "logger", (startUpFunctor_t)startUpLogger);
}

//eof

