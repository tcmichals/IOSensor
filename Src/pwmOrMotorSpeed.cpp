


#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <algorithm>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"

#include "msgLogger.h"
#include "ringBuffer.h"
#include "msgLogger.h"
#include "startUp.h"

#include "pwmOrMotorSpeed.h"





#define NUM_MSGS 10

//Used as a watchdog and 
static void pwmOrMotorSpeedTask(void *arg)
{
    
    static StaticSemaphore_t xStaticSema;
    static SemaphoreHandle_t xSemaHandle;

    xSemaHandle = xSemaphoreCreateBinaryStatic(&xStaticSema);
    TickType_t ticks = 1000 / portTICK_PERIOD_MS;
    
    
    while(1)
    {
        /* find sensor */
          if(xSemaphoreTake(xSemaHandle, (TickType_t)ticks) == pdTRUE)
           {
           }
    
    }
    
    
}


#define STACK_SIZE_TASK 512
#define PRIORITY_PHANDLER (tskIDLE_PRIORITY + 1)

static StaticTask_t _gTskHandleStatic;
static TaskHandle_t _gTskHandle;
static std::array<StackType_t, STACK_SIZE_TASK> _stack;


static bool startUpPWMOrMotorSpeed(void)
{

    _gTskHandle = xTaskCreateStatic(pwmOrMotorSpeedTask,
                                        "pwmOrMotorSpeed",
                                        STACK_SIZE_TASK,
                                        (void*)nullptr,
                                        PRIORITY_PHANDLER,
                                        (StackType_t*)_stack.data(),
                                        &_gTskHandleStatic);

    if(_gTskHandle)
    {
        return true;
    }
    else
    {
        return false;
    }
}



static void pwmOrMotorSpeedTskConstruct(void) __attribute__((constructor(102)));
void pwmOrMotorSpeedTskConstruct(void)
{
    regStartUpFunction(StartUpPriority::Last, "", (startUpFunctor_t)startUpPWMOrMotorSpeed);
}

/* start task */



