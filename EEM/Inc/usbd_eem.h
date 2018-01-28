/**
  ******************************************************************************
  * @file    usbd_eem.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   header file for the usbd_eem.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_EEM_H
#define __USB_EEM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup usbd_eem
  * @brief This file is the Header file for usbd_eem.c
  * @{
  */ 


/** @defgroup usbd_eem_Exported_Defines
  * @{
  */ 
#define EEM_IN_EP                                   0x81  /* EP1 for data IN */
#define EEM_OUT_EP                                  0x01  /* EP1 for data OUT */


/* EEM Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define EEM_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define EEM_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define EEM_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */ 

#define USB_EEM_CONFIG_DESC_SIZ                     67
#define EEM_DATA_HS_IN_PACKET_SIZE                  EEM_DATA_HS_MAX_PACKET_SIZE
#define EEM_DATA_HS_OUT_PACKET_SIZE                 EEM_DATA_HS_MAX_PACKET_SIZE

#define EEM_DATA_FS_IN_PACKET_SIZE                  EEM_DATA_FS_MAX_PACKET_SIZE
#define EEM_DATA_FS_OUT_PACKET_SIZE                 EEM_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  EEM definitions                                                    */
/*---------------------------------------------------------------------*/

#define EEM_IS_COMMAND                  0x8000
#define EEM_IS_PKT_CRC                  0x4000
#define EEM_PKT_LEN_MASK                0x3FFF

#define EEM_CMD_IS_RESERVED             (1 <<14)
#define EEM_CMD_MASK                    0x3
#define EEM_CMD_ECHO                    0x00 ///bmEEMCmd Echo
#define EEM_CMD_ECHO_RESPONSE           0x01 ///bmEEMCmd Echo Response
#define EEM_CMD_SUSPEND_HINT            0x02 ///bmEEMCmd Suspend Hint
#define EEM_CMD_RESPONSE_HINT           0x03 ///bmEEMCmd Response Hint
#define EEM_CMD_RESPONSE_COMPLETE_HINT  0x04 ///bmEEMCmd Response Complete Hint
#define EEM_CMD_TICKLE                  0x05 ///bmEEMCmd Tickle

#define USB_CLASS_COMM			2
#define USB_CDC_SUBCLASS_EEM		0x0c
#define USB_CDC_PROTO_EEM			7


/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */ 

#define EEM_PKT_STATE_START 0
#define EEM_PKT_STATE_MIDDLE 1
typedef struct _USBD_EEM_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t);   
  int8_t (* Receive)       (uint8_t *, uint32_t *);  
  int8_t (* TxDone)        (uint8_t *, uint32_t);

}USBD_EEM_ItfTypeDef;


typedef struct
{
  uint32_t data[EEM_DATA_HS_MAX_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;    
  uint8_t  *RxBuffer;  
  uint8_t  *TxBuffer;   
  uint32_t RxLength;
  uint32_t TxLength;    
  
  __IO uint32_t TxState;     
  __IO uint32_t RxState;
}
USBD_EEM_HandleTypeDef; 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 
  
/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_ClassTypeDef  USBD_EEM;
#define USBD_EEM_CLASS    &USBD_EEM
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_EEM_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      USBD_EEM_ItfTypeDef *fops);

uint8_t  USBD_EEM_SetTxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff,
                                      uint16_t length);

uint8_t  USBD_EEM_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);
  
uint8_t  USBD_EEM_ReceivePacket      (USBD_HandleTypeDef *pdev);

uint8_t  USBD_EEM_TransmitPacket     (USBD_HandleTypeDef *pdev);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_EEM_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
