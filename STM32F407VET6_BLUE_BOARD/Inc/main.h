/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PPS_GPS_Pin GPIO_PIN_1
#define PPS_GPS_GPIO_Port GPIOC
#define PPS_GPS_EXTI_IRQn EXTI1_IRQn
#define BAT_IN_Pin GPIO_PIN_0
#define BAT_IN_GPIO_Port GPIOA
#define LED_BAR_CLOCK_Pin GPIO_PIN_5
#define LED_BAR_CLOCK_GPIO_Port GPIOA
#define LED_BAR_DATA_Pin GPIO_PIN_7
#define LED_BAR_DATA_GPIO_Port GPIOA
#define M1_Pin GPIO_PIN_9
#define M1_GPIO_Port GPIOE
#define M2_Pin GPIO_PIN_11
#define M2_GPIO_Port GPIOE
#define M3_Pin GPIO_PIN_13
#define M3_GPIO_Port GPIOE
#define M4_Pin GPIO_PIN_14
#define M4_GPIO_Port GPIOE
#define CH1_Pin GPIO_PIN_10
#define CH1_GPIO_Port GPIOD
#define CH1_EXTI_IRQn EXTI15_10_IRQn
#define CH2_Pin GPIO_PIN_11
#define CH2_GPIO_Port GPIOD
#define CH2_EXTI_IRQn EXTI15_10_IRQn
#define CH3_Pin GPIO_PIN_12
#define CH3_GPIO_Port GPIOD
#define CH3_EXTI_IRQn EXTI15_10_IRQn
#define CH4_Pin GPIO_PIN_13
#define CH4_GPIO_Port GPIOD
#define CH4_EXTI_IRQn EXTI15_10_IRQn
#define CH5_Pin GPIO_PIN_14
#define CH5_GPIO_Port GPIOD
#define CH5_EXTI_IRQn EXTI15_10_IRQn
#define CH6_Pin GPIO_PIN_15
#define CH6_GPIO_Port GPIOD
#define CH6_EXTI_IRQn EXTI15_10_IRQn
#define DEBUG_TX_Pin GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
