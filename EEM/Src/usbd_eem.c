/**
  ******************************************************************************
  * @file    usbd_eem.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB EEM Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as eem Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                EEM Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as EEM device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *      
  *  @endverbatim
  *                                  
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_eem.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


static USBD_EEM_HandleTypeDef gUSB_EEMInstance;
/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_EEM 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_EEM_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_EEM_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_EEM_Private_Macros
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_EEM_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_EEM_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_EEM_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_EEM_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  USBD_EEM_DataIn (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_EEM_DataOut (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_EEM_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_EEM_GetFSCfgDesc (uint16_t *length);

static uint8_t  *USBD_EEM_GetHSCfgDesc (uint16_t *length);

static uint8_t  *USBD_EEM_GetOtherSpeedCfgDesc (uint16_t *length);

static uint8_t  *USBD_EEM_GetOtherSpeedCfgDesc (uint16_t *length);

uint8_t  *USBD_EEM_GetDeviceQualifierDescriptor (uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_EEM_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_EEM_Private_Variables
  * @{
  */ 


/* EEM interface class callbacks structure */
USBD_ClassTypeDef  USBD_EEM = 
{
  USBD_EEM_Init,
  USBD_EEM_DeInit,
  USBD_EEM_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_EEM_EP0_RxReady,
  USBD_EEM_DataIn,
  USBD_EEM_DataOut,
  NULL,
  NULL,
  NULL,     
  USBD_EEM_GetHSCfgDesc,  
  USBD_EEM_GetFSCfgDesc,    
  USBD_EEM_GetOtherSpeedCfgDesc, 
  USBD_EEM_GetDeviceQualifierDescriptor,
};

/* USB EEM device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_EEM_CfgHSDesc[USB_EEM_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   				/* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      	/* bDescriptorType: Configuration */
  USB_EEM_CONFIG_DESC_SIZ,         	/* wTotalLength:no of returned bytes */
  0x00,
  0x01,   				/* bNumInterfaces: 1 interface */
  0x01,   				/* bConfigurationValue: Configuration value */
  0x00,   				/* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   				/* bmAttributes: self powered */
  0x32,   				/* MaxPower 0 mA */
  
  /*---------------------------------------------------------------------------*/
  
  /*Interface Descriptor */
  0x09,   				/* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  		/* bDescriptorType: Interface */
					/* Interface descriptor type */
  0x00,   				/* bInterfaceNumber: Number of Interface */
  0x00,   				/* bAlternateSetting: Alternate setting */
  0x02,   				/* bNumEndpoints: One endpoints used */
  USB_CLASS_COMM,   			/* bInterfaceClass: Communication Interface Class */
  USB_CDC_SUBCLASS_EEM,   		/* bInterfaceSubClass: Abstract Control Model */
  USB_CDC_PROTO_EEM,   			/* bInterfaceProtocol: Common AT commands */
  0x00,   				/* iInterface: */
    
  /*Endpoint OUT Descriptor*/
  0x07,   				/* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      		/* bDescriptorType: Endpoint */
  EEM_OUT_EP,                        	/* bEndpointAddress */
  0x02,                              	/* bmAttributes: Bulk */
  LOBYTE(EEM_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(EEM_DATA_HS_MAX_PACKET_SIZE),
  0x00,                              	/* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   				/* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      		/* bDescriptorType: Endpoint */
  EEM_IN_EP,                         	/* bEndpointAddress */
  0x02,                              	/* bmAttributes: Bulk */
  LOBYTE(EEM_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(EEM_DATA_HS_MAX_PACKET_SIZE),
  0x00                               	/* bInterval: ignore for Bulk transfer */
} ;


/* USB EEM device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_EEM_CfgFSDesc[USB_EEM_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   				/* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      	/* bDescriptorType: Configuration */
  USB_EEM_CONFIG_DESC_SIZ,              /* wTotalLength:no of returned bytes */
  0x00,
  0x01,   				/* bNumInterfaces: 1 interface */
  0x01,   				/* bConfigurationValue: Configuration value */
  0x00,   				/* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   				/* bmAttributes: self powered */
  0x32,   				/* MaxPower 0 mA */
  
  /*---------------------------------------------------------------------------*/
  
  /*Interface Descriptor */
  0x09,   				/* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  		/* bDescriptorType: Interface */
  					/* Interface descriptor type */
  0x00,   				/* bInterfaceNumber: Number of Interface */
  0x00,   				/* bAlternateSetting: Alternate setting */
  0x02,   				/* bNumEndpoints: 2 endpoints used */
  USB_CLASS_COMM,   			/* bInterfaceClass: Communication Interface Class */
  USB_CDC_SUBCLASS_EEM,   		/* bInterfaceSubClass: USB_CDC_SUBCLASS_EEM */
  USB_CDC_PROTO_EEM,   			/* bInterfaceProtocol: USB_CDC_PROTO_EEM */
  0x00,   				/* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   				/* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      		/* bDescriptorType: Endpoint */
  EEM_OUT_EP,                        	/* bEndpointAddress */
  0x02,                              	/* bmAttributes: Bulk */
  LOBYTE(EEM_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(EEM_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              	/* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   				/* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      		/* bDescriptorType: Endpoint */
  EEM_IN_EP,                         	/* bEndpointAddress */
  0x02,                              	/* bmAttributes: Bulk */
  LOBYTE(EEM_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(EEM_DATA_FS_MAX_PACKET_SIZE),
  0x00                               	/* bInterval: ignore for Bulk transfer */
} ;

__ALIGN_BEGIN uint8_t USBD_EEM_OtherSpeedCfgDesc[USB_EEM_CONFIG_DESC_SIZ] __ALIGN_END =
{ 
  0x09,   				/* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,   
  USB_EEM_CONFIG_DESC_SIZ,
  0x00,
  0x02,   				/* bNumInterfaces: 2 interfaces */
  0x01,   				/* bConfigurationValue: */
  0x04,   				/* iConfiguration: */
  0xC0,   				/* bmAttributes: */
  0x32,   				/* MaxPower 100 mA */  
  
  /*Interface Descriptor */
  0x09,   				/* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  		/* bDescriptorType: Interface */
  					/* Interface descriptor type */
  0x00,   				/* bInterfaceNumber: Number of Interface */
  0x00,   				/* bAlternateSetting: Alternate setting */
  0x02,   				/* bNumEndpoints: One endpoints used */
  USB_CLASS_COMM,   			/* bInterfaceClass: Communication Interface Class */
  USB_CDC_SUBCLASS_EEM,  		/* bInterfaceSubClass: Abstract Control Model */
  USB_CDC_PROTO_EEM,   			/* bInterfaceProtocol: Common AT commands */
  0x00,   				/* iInterface: */
  
   
  /*Endpoint OUT Descriptor*/
  0x07,   				/* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      		/* bDescriptorType: Endpoint */
  EEM_OUT_EP,                        	/* bEndpointAddress */
  0x02,                              	/* bmAttributes: Bulk */
  0x40,                              	/* wMaxPacketSize: */
  0x00,
  0x00,                              	/* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   				/* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,     		/* bDescriptorType: Endpoint */
  EEM_IN_EP,                        	/* bEndpointAddress */
  0x02,                    		/* bmAttributes: Bulk */
  0x40,                             	/* wMaxPacketSize: */
  0x00,
  0x00                              	/* bInterval */
};

/**
  * @}
  */ 

/** @defgroup USBD_EEM_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_EEM_Init
  *         Initialize the EEM interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_EEM_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_EEM_HandleTypeDef   *heem;
  
  if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
  {  
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
                   EEM_IN_EP,
                   USBD_EP_TYPE_BULK,
                   EEM_DATA_HS_IN_PACKET_SIZE);
    
    /* Open EP OUT */
    USBD_LL_OpenEP(pdev,
                   EEM_OUT_EP,
                   USBD_EP_TYPE_BULK,
                   EEM_DATA_HS_OUT_PACKET_SIZE);
    
  }
  else
  {
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
                   EEM_IN_EP,
                   USBD_EP_TYPE_BULK,
                   EEM_DATA_FS_IN_PACKET_SIZE);
    
    /* Open EP OUT */
    USBD_LL_OpenEP(pdev,
                   EEM_OUT_EP,
                   USBD_EP_TYPE_BULK,
                   EEM_DATA_FS_OUT_PACKET_SIZE);
  }
  
 memset(&gUSB_EEMInstance, 0, sizeof(gUSB_EEMInstance));
  pdev->pClassData = &gUSB_EEMInstance; //USBD_malloc(sizeof (USBD_EEM_HandleTypeDef));
  
  if(pdev->pClassData == NULL)
  {
    ret = 1; 
  }
  else
  {
    heem = (USBD_EEM_HandleTypeDef*) pdev->pClassData;
    
    /* Init  physical Interface components */
    ((USBD_EEM_ItfTypeDef *)pdev->pUserData)->Init();
    
    /* Init Xfer states */
    heem->TxState =0;
    heem->RxState =0;

       
    if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
    {      
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             EEM_OUT_EP,
                             heem->RxBuffer,
                             EEM_DATA_HS_OUT_PACKET_SIZE);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             EEM_OUT_EP,
                             heem->RxBuffer,
                             EEM_DATA_FS_OUT_PACKET_SIZE);
    }
    
    
  }
  return ret;
}

/**
  * @brief  USBD_EEM_Init
  *         DeInitialize the EEM layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_EEM_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  uint8_t ret = 0;
  
  /* Open EP IN */
  USBD_LL_CloseEP(pdev,
              EEM_IN_EP);
  
  /* Open EP OUT */
  USBD_LL_CloseEP(pdev,
              EEM_OUT_EP);
  
  
  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    ((USBD_EEM_ItfTypeDef *)pdev->pUserData)->DeInit();
   //TCM  USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  
  return ret;
}

/**
  * @brief  USBD_EEM_Setup
  *         Handle the EEM specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_EEM_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  USBD_EEM_HandleTypeDef   *heem = (USBD_EEM_HandleTypeDef*) pdev->pClassData;
  static uint8_t ifalt = 0;
    
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    if (req->wLength)
    {
      if (req->bmRequest & 0x80)
      {
        ((USBD_EEM_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                          (uint8_t *)heem->data,
                                                          req->wLength);
          USBD_CtlSendData (pdev, 
                            (uint8_t *)heem->data,
                            req->wLength);
      }
      else
      {
        heem->CmdOpCode = req->bRequest;
        heem->CmdLength = req->wLength;
        
        USBD_CtlPrepareRx (pdev, 
                           (uint8_t *)heem->data,
                           req->wLength);
      }
      
    }
    else
    {
      ((USBD_EEM_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                        (uint8_t*)req,
                                                        0);
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        &ifalt,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      break;
    }
 
  default: 
    break;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_EEM_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_EEM_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_EEM_HandleTypeDef   *heem = (USBD_EEM_HandleTypeDef*) pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    
    if (((USBD_EEM_ItfTypeDef *)pdev->pUserData)->TxDone != NULL)
         ((USBD_EEM_ItfTypeDef *)pdev->pUserData)->TxDone(heem->TxBuffer, heem->TxLength);
    
    heem->TxState = 0;

    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_EEM_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_EEM_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
  USBD_EEM_HandleTypeDef   *heem = (USBD_EEM_HandleTypeDef*) pdev->pClassData;
  
  /* Get the received data length */
  heem->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the application Xfer */
  if(pdev->pClassData != NULL)
  {
    ((USBD_EEM_ItfTypeDef *)pdev->pUserData)->Receive(heem->RxBuffer, &heem->RxLength);

    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}



/**
  * @brief  USBD_EEM_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_EEM_EP0_RxReady (USBD_HandleTypeDef *pdev)
{ 
  USBD_EEM_HandleTypeDef   *heem = (USBD_EEM_HandleTypeDef*) pdev->pClassData;
  
  if((pdev->pUserData != NULL) && (heem->CmdOpCode != 0xFF))
  {
    ((USBD_EEM_ItfTypeDef *)pdev->pUserData)->Control(heem->CmdOpCode,
                                                      (uint8_t *)heem->data,
                                                      heem->CmdLength);
      heem->CmdOpCode = 0xFF; 
      
  }
  return USBD_OK;
}

/**
  * @brief  USBD_EEM_GetFSCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_EEM_GetFSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_EEM_CfgFSDesc);
  return USBD_EEM_CfgFSDesc;
}

/**
  * @brief  USBD_EEM_GetHSCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_EEM_GetHSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_EEM_CfgHSDesc);
  return USBD_EEM_CfgHSDesc;
}

/**
  * @brief  USBD_EEM_GetCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_EEM_GetOtherSpeedCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_EEM_OtherSpeedCfgDesc);
  return USBD_EEM_OtherSpeedCfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_EEM_GetDeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_EEM_DeviceQualifierDesc);
  return USBD_EEM_DeviceQualifierDesc;
}

/**
* @brief  USBD_EEM_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t  USBD_EEM_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      USBD_EEM_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;
  
  if(fops != NULL)
  {
    pdev->pUserData= fops;
    ret = USBD_OK;    
  }
  
  return ret;
}

/**
  * @brief  USBD_EEM_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_EEM_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length)
{
  USBD_EEM_HandleTypeDef   *heem = (USBD_EEM_HandleTypeDef*) pdev->pClassData;
  
  heem->TxBuffer = pbuff;
  heem->TxLength = length;  
  
  return USBD_OK;  
}


/**
  * @brief  USBD_EEM_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_EEM_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t  *pbuff)
{
  USBD_EEM_HandleTypeDef   *heem = (USBD_EEM_HandleTypeDef*) pdev->pClassData;
  
  heem->RxBuffer = pbuff;
  
  return USBD_OK;
}

/**
  * @brief  USBD_EEM_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_EEM_TransmitPacket(USBD_HandleTypeDef *pdev)
{      
  USBD_EEM_HandleTypeDef   *heem = (USBD_EEM_HandleTypeDef*) pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    if(heem->TxState == 0)
    {
      /* Tx Transfer in progress */
      heem->TxState = 1;
      
      /* Transmit next packet */
      USBD_LL_Transmit(pdev,
                       EEM_IN_EP,
                       heem->TxBuffer,
                       heem->TxLength);
      
      return USBD_OK;
    }
    else
    {
      return USBD_BUSY;
    }
  }
  else
  {
    return USBD_FAIL;
  }
}


/**
  * @brief  USBD_EEM_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_EEM_ReceivePacket(USBD_HandleTypeDef *pdev)
{      
  USBD_EEM_HandleTypeDef   *heem = (USBD_EEM_HandleTypeDef*) pdev->pClassData;
  
  /* Suspend or Resume USB Out process */
  if(pdev->pClassData != NULL)
  {
    if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
    {      
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             EEM_OUT_EP,
                             heem->RxBuffer,
                             EEM_DATA_HS_OUT_PACKET_SIZE);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             EEM_OUT_EP,
                             heem->RxBuffer,
                             EEM_DATA_FS_OUT_PACKET_SIZE);
    }
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
