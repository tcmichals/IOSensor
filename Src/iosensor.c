
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "msgLogger.h"

extern int hwInit(void); 
extern bool isUSBLinkActive(void);

osThreadId defaultTaskHandle;
void StartDefaultTask(void const * argument);

const int uxTopUsedPriority = configMAX_PRIORITIES;

void lwipPrintf(const char *pFormat, ...)
{
	char debugStr[128];

	memset(debugStr, 0, sizeof(debugStr));

	va_list aptr;
	va_start(aptr, pFormat);
	int len = vsnprintf(debugStr, sizeof(debugStr), pFormat, aptr);
	va_end(aptr);

	logMsg(LogMsg_Debug, debugStr);

}

void StartDefaultTask(void const * argument)
{
    
  GPIO_InitTypeDef GPIO_InitStruct;
   /*Configure GPIO pin : DP  */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
 osDelay(50);
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
 osDelay(1);
 
  /* init code for USB_DEVICE */
    MX_LWIP_Init();
  MX_USB_DEVICE_Init();
  callStartupFunctions();

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    if(isUSBLinkActive())
    {
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
        osDelay(100);
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
         osDelay(100);
    }
  }
  /* USER CODE END 5 */ 
}




int main(void)
{

    hwInit();
    
      /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */


}

//eof
