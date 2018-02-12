#if 0

#include "gd32f10x.h"
#include <stdio.h>
#include "systick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"




/*
 * PB9 - PWM - OUT TM4_CH4 ALT 0
 * PB8 - PWM - OUT TM4_CH3 ALT 0
 * PB7 - PWM - OUT TM4_CH2 ALT 1 //AFIO_PCFR1_I2C1_REMAP
 * PB6 - PWM - OUT TM4_CH1 ALT 1 // //AFIO_PCFR1_I2C1_REMAP
 *
 * I2C1 is now on SCL/PB8, SDA/PB9
 */

void HAL_PWM_MspInit()
{

    //Allows
    GPIO_PinRemapConfig(GPIO_REMAP_I2C1, ENABLE);


}



void HAL_PWM_Write(int device, int val)
{





}

#endif 