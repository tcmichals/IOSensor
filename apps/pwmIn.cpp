
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "pwmIn.h"

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

#include "msgProtocolPortsNMsgs.h"
#include "msgProtocol.h"


#define NUM_MSGS 10
#define STACK_SIZE_PHANDLER 512
#define PRIORITY_PHANDLER (tskIDLE_PRIORITY + 1)

static StaticTask_t gPWMInTskHandleStatic;
static TaskHandle_t gPWMInTskHandle;
static std::array<StackType_t, STACK_SIZE_PHANDLER> stackPHandlerPWMIn;

#define NUM_PWM_IN_CHANNELS 6u
#define FAILSAFE_DETECT_TRESHOLD 985

static uint16_t pwmInput[NUM_PWM_IN_CHANNELS] = { 1500, 1500, 1500, 1500, 1500, 1500 };
static uint16_t GoodPulses;

static StaticSemaphore_t xPWMInReadySemaBuffer;
static SemaphoreHandle_t xPWMInReadySema;
static bool pwmInSetup;

extern TIM_HandleTypeDef htim14;

typedef struct
{
	msgProtocol::pwmInMsgHdr_t m_hdr;
	msgProtocol::pwmInChannelMsg_t pwmMsgHdr;
    uint16_t pwmChannels[NUM_PWM_IN_CHANNELS];

} __attribute__((packed)) localPWMIn_t;


static bool _gRadioLinkIsActive;

bool isRadioLinkActive()
{
	return _gRadioLinkIsActive;
}

void updatePWMChannelFromISR(int pin, GPIO_PinState value, uint16_t timerVal)
{
    if(false == pwmInSetup)
        return;

    if(value != GPIO_PIN_SET)
    {
        if(900 < timerVal && timerVal < 2200)
        {
            pwmInput[pin] = timerVal;
            if(timerVal > FAILSAFE_DETECT_TRESHOLD)
            {
                GoodPulses |= (1 << pin);
            }
        }
    }
    else
    {
        __HAL_TIM_SET_COUNTER(&htim14, 0);
    }

    if(GoodPulses == ALL_PWM_IN)
    {
        GoodPulses = 0;
        portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xPWMInReadySema, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void pwmInTask(void* arg)
{
    (void)arg;
    // need to generate PWM messages...
    xPWMInReadySema = xSemaphoreCreateBinaryStatic(&xPWMInReadySemaBuffer);

    pwmInSetup = true;
    TickType_t ticks = 1000 / portTICK_PERIOD_MS;
    localPWMIn_t pkt;

    memset(&pkt, 0, sizeof(pkt));

    pkt.m_hdr.m_msgType = msgProtocol::pwmInMsg::pwmInChannelsMsg;
    pkt.m_hdr.m_lenOfMsg = sizeof(pkt.pwmMsgHdr) +  sizeof(pkt.pwmChannels);

    pkt.pwmMsgHdr.numOfChannels= NUM_PWM_IN_CHANNELS;
    std::fill_n(pkt.pwmChannels, NUM_PWM_IN_CHANNELS, 0xFFFF);

    HAL_TIM_Base_Start(&htim14);

    while(1)
    {
        if(xSemaphoreTake(xPWMInReadySema, (TickType_t)ticks) == pdTRUE)
        {
            memcpy(pkt.pwmChannels, pwmInput, sizeof(pkt.pwmChannels));

            postMsg(msgProtocol::msgPorts::pwmInClient,
            		msgProtocol::msgPorts::pwmInServer,
            		&pkt,
					sizeof(pkt));
            pkt.pwmMsgHdr.counter++;
            _gRadioLinkIsActive = true;
        }
        else // time out occurred
        {
            std::fill_n(pkt.pwmChannels, NUM_PWM_IN_CHANNELS, 0xFFFF);
            postMsg(msgProtocol::msgPorts::pwmInClient,
            		msgProtocol::msgPorts::pwmInServer,
            		&pkt,
					sizeof(pkt));
            pkt.pwmMsgHdr.counter++;
            _gRadioLinkIsActive = false;
        }
    }
}

static bool startUpPWMIN(void)
{

    gPWMInTskHandle = xTaskCreateStatic(pwmInTask,
                                        "pwmIn",
                                        STACK_SIZE_PHANDLER,
                                        (void*)nullptr,
                                        PRIORITY_PHANDLER,
                                        (StackType_t*)stackPHandlerPWMIn.data(),
                                        &gPWMInTskHandleStatic);

    if(gPWMInTskHandle)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void _startUpPWMIn(void) __attribute__((constructor(102)));
void _startUpPWMIn(void)
{
    regStartUpFunction(StartUpPriority::Last, "pwmIn", (startUpFunctor_t)startUpPWMIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    register uint16_t _pin = 0;
    register GPIO_PinState val = HAL_GPIO_ReadPin(GPIOD, GPIO_Pin);

    switch(GPIO_Pin)
    {

    case GPIO_PIN_10:
        _pin = 1;
        break;
    case GPIO_PIN_11:
        _pin = 2;
        break;
    case GPIO_PIN_12:
        _pin = 3;
        break;
    case GPIO_PIN_13:
        _pin = 4;
        break;
    case GPIO_PIN_14:
        _pin = 5;
        break;
    case GPIO_PIN_15:
        _pin = 6;
        break;
    }

    if(_pin)
        updatePWMChannelFromISR(_pin - 1, val, __HAL_TIM_GET_COUNTER(&htim14));
}
// post message when done..

