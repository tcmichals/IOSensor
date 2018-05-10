/**
  ******************************************************************************
  * @file           : usbd_eem_if.c
  * @brief          :
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 Tim Michals
  * All rights reserved.
  *
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <algorithm>
#include <cstring>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "usbd_eem_if.h"
extern "C" {
#include "lwip/sys.h"    
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "ethernetif.h"
#include <string.h>
#include "cmsis_os.h"
}

#include "ringBuffer.h"
#include "msgLogger.h"

/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_EEM
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_EEM_Private_TypesDefinitions
  * @{
  */
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */
/**
  * @}
  */

/** @defgroup USBD_EEM_Private_Defines
  * @{
  */
/* USER CODE BEGIN PRIVATE_DEFINES */

static bool _gIsUSBLinkActive;
#define FULL_SPEED_SIZE 64
#define RING_TX_SIZE 10
#define INTERFACE_THREAD_STACK_SIZE 1024

#define MAX_EEM_POOL 4
__ALIGN_BEGIN uint8_t __attribute__((section(".ccmram")))  Rx_Buff[MAX_EEM_POOL][EEM_DATA_HS_OUT_PACKET_SIZE]; __ALIGN_END
uint8_t __attribute__((section(".ccmram")))  txEEMPkt[1600];

typedef struct
{
    struct pbuf* p;
    int pktState;
    int pktLen;
    int pktToGo;
    uint8_t* pPkt;
} eem_pktInfo_t;

static QueueHandle_t pbufQueue;

ringBuffer<struct pbuf*> eemTxRingBuffer;
static int gWorkPkt;

enum class lwipRequest : int
{
    lwipRX_t,
    lwipTxStart_t,
    eemUSBTxDone_t,
    lwipLink_t
};

typedef struct
{
    lwipRequest request;
    int pktSize;
    uint8_t* pPkt;
    struct pbuf* pTxPkt;
} lwipMsg_t;

/* USER CODE END PRIVATE_DEFINES */
/**
  * @}
  */

/** @defgroup USBD_EEM_Private_Macros
  * @{
  */
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_EEM_Private_Variables
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_EEM_IF_Exported_Variables
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern struct netif gnetif;
;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_EEM_Private_FunctionPrototypes
  * @{
  */
static int8_t EEM_Init_FS(void);
static int8_t EEM_DeInit_FS(void);
static int8_t EEM_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t EEM_Receive_FS(uint8_t* pbuf, uint32_t* Len);
static int8_t EEM_TxDone_FS(uint8_t*, uint32_t len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_EEM_ItfTypeDef USBD_Interface_fops_FS = { EEM_Init_FS, EEM_DeInit_FS, EEM_Control_FS, EEM_Receive_FS, EEM_TxDone_FS

};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  EEM_Init_FS
  *         Initializes the EEM media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t EEM_Init_FS(void)
{
    /* USER CODE BEGIN 3 */
    /* Set Application Buffers */
    BaseType_t xHigherPriorityTaskWoken = false;

    _gIsUSBLinkActive = true;
    gWorkPkt = 0;
    USBD_EEM_SetRxBuffer(&hUsbDeviceFS, &Rx_Buff[gWorkPkt][0]);
    gWorkPkt++;
    if ( gWorkPkt >= MAX_EEM_POOL)
        gWorkPkt =0;

    lwipMsg_t lwipMsg;
    lwipMsg.request = lwipRequest::lwipLink_t;
    lwipMsg.pktSize = 0;
    lwipMsg.pPkt = nullptr;
    lwipMsg.pTxPkt = nullptr;

    xQueueSendToBackFromISR(pbufQueue, &lwipMsg, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return (USBD_OK);
    /* USER CODE END 3 */
}

/**
  * @brief  EEM_DeInit_FS
  *         DeInitializes the EEM media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t EEM_DeInit_FS(void)
{
    /* USER CODE BEGIN 4 */
    _gIsUSBLinkActive = false;
    return (USBD_OK);
    /* USER CODE END 4 */
}

/**
  * @brief  EEM_Control_FS
  *         Manage the EEM class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t EEM_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    /* USER CODE BEGIN 5 */
    (void)pbuf;
    (void)length;
    switch(cmd)
    {

    default:
        break;
    }

    return (USBD_OK);
    /* USER CODE END 5 */
}

/**
  * @brief  EEM_Receive_FS
  *         Data received over USB OUT endpoint are sent over EEM interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on EEM interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t EEM_Receive_FS(uint8_t* Buf, uint32_t* Len)
{

    BaseType_t xHigherPriorityTaskWoken = false;
    lwipMsg_t lwipMsg;

    lwipMsg.request = lwipRequest::lwipRX_t;
    lwipMsg.pktSize = *Len;
    lwipMsg.pPkt = Buf;

    xQueueSendToBackFromISR(pbufQueue, &lwipMsg, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return (USBD_OK);
}

int8_t EEM_TxDone_FS(uint8_t* pPkt, uint32_t len)
{
    BaseType_t xHigherPriorityTaskWoken = false;
    lwipMsg_t lwipMsg;

    lwipMsg.request = lwipRequest::eemUSBTxDone_t;
    lwipMsg.pktSize = len;
    lwipMsg.pPkt = pPkt;

    xQueueSendToBackFromISR(pbufQueue, &lwipMsg, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return (USBD_OK);
}

/**
  * @brief  EEM_Transmit_FS
  *         Data send over USB IN endpoint are sent over EEM interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be send
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t EEM_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
    uint8_t result = USBD_OK;

    USBD_EEM_HandleTypeDef* heem = (USBD_EEM_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if(heem->TxState != 0)
    {
        return USBD_BUSY;
    }
    USBD_EEM_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    result = USBD_EEM_TransmitPacket(&hUsbDeviceFS);

    return result;
}

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif* netif)
{

    /* Init ETH */
    uint8_t MACAddr[6];

    MACAddr[0] = 0x00;
    MACAddr[1] = 0x80;
    MACAddr[2] = 0xE1;
    MACAddr[3] = 0x00;
    MACAddr[4] = 0x00;
    MACAddr[5] = 0x00;

    if(_gIsUSBLinkActive == true)
    {
        /* Set netif link flag */
        netif->flags |= NETIF_FLAG_LINK_UP;
    }

#if LWIP_ARP || LWIP_ETHERNET

    /* set MAC hardware address length */
    netif->hwaddr_len = ETH_HWADDR_LEN;

    /* set MAC hardware address */
    netif->hwaddr[0] = MACAddr[0];
    netif->hwaddr[1] = MACAddr[1];
    netif->hwaddr[2] = MACAddr[2];
    netif->hwaddr[3] = MACAddr[3];
    netif->hwaddr[4] = MACAddr[4];
    netif->hwaddr[5] = MACAddr[5];

    /* maximum transfer unit */
    netif->mtu = 1500;

/* Accept broadcast address and ARP traffic */
/* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
#if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
#else
    netif->flags |= NETIF_FLAG_BROADCAST;
#endif /* LWIP_ARP */

    pbufQueue = xQueueCreate(RING_TX_SIZE, sizeof(lwipMsg_t));

    /* create the task that handles the ETH_MAC */
    osThreadDef(EthIf, ethernetif_input, osPriorityRealtime, 0, INTERFACE_THREAD_STACK_SIZE);
    osThreadCreate(osThread(EthIf), netif);
/* Enable MAC and DMA transmission and reception */

/* USER CODE BEGIN PHY_PRE_CONFIG */

/* USER CODE END PHY_PRE_CONFIG */

/* USER CODE BEGIN PHY_POST_CONFIG */

/* USER CODE END PHY_POST_CONFIG */

#endif /* LWIP_ARP || LWIP_ETHERNET */

    /* USER CODE BEGIN LOW_LEVEL_INIT */

    /* USER CODE END LOW_LEVEL_INIT */
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif* netif, struct pbuf* p)
{
    
    (void)(netif);
    
    if(false == _gIsUSBLinkActive)
        return ERR_MEM;

    err_t rc = ERR_OK;
    /* USER CODE BEGIN 7 */

    lwipMsg_t lwipMsg;

    lwipMsg.request = lwipRequest::lwipTxStart_t;
    lwipMsg.pktSize = 0;
    lwipMsg.pPkt = nullptr;
    lwipMsg.pTxPkt = p;

    pbuf_ref(p);
    if(!xQueueSend(pbufQueue, &lwipMsg, 0))
    {
        logMsg(LogMsg_Debug, "ERROR %s %d p->ref=%d", __PRETTY_FUNCTION__, __LINE__, p->ref);
        rc = ERR_MEM;
	//need to remove the reference 
	pbuf_free(p);
    }


    return rc; // errval;
}



static bool localTx(lwipMsg_t &lwipMsg, eem_pktInfo_t &eemTxPkt)
{
    
    
    memset(&eemTxPkt, 0, sizeof(eemTxPkt));
    eemTxPkt.pktLen = lwipMsg.pTxPkt->tot_len + sizeof(uint16_t) + sizeof(uint32_t);
    uint32_t lenCopied = pbuf_copy_partial(lwipMsg.pTxPkt, txEEMPkt + sizeof(uint16_t), lwipMsg.pTxPkt->tot_len, 0);
            
    pbuf_free(lwipMsg.pTxPkt);
            
    lwipMsg.pTxPkt = nullptr;

    uint16_t* pTxEEMHdr = reinterpret_cast<uint16_t*>(txEEMPkt);

    *pTxEEMHdr = lenCopied + sizeof(uint32_t);
    eemTxPkt.pPkt = txEEMPkt;

    uint8_t* pCRC = txEEMPkt + lenCopied + sizeof(uint16_t);
    *pCRC++ = 0xde;
    *pCRC++ = 0xad;
    *pCRC++ = 0xbe;
    *pCRC++ = 0xef;

    USBD_EEM_SetTxBuffer(&hUsbDeviceFS, txEEMPkt, eemTxPkt.pktLen);
    if(USBD_OK != USBD_EEM_TransmitPacket(&hUsbDeviceFS))
    {
        logMsg(LogMsg_Debug, "%s %d lwipTxStart_t error", __PRETTY_FUNCTION__, __LINE__);
        return false;
    }
    return true;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input(void const* argument)
{
    struct netif* netif = (struct netif*)argument;
    eem_pktInfo_t eemRx;
    eem_pktInfo_t eemTxPkt;

    memset(&eemRx, 0, sizeof(eemRx));
    memset(&eemTxPkt, 0, sizeof(eemTxPkt));

    for(;;)
    {
        lwipMsg_t lwipMsg;
        BaseType_t evt = xQueueReceive(pbufQueue, (void*)&lwipMsg, 1000);

        if(false == evt)
        {
            continue;
        }

        switch(lwipMsg.request)
        {

        case lwipRequest::lwipRX_t:
        {

            int remainingBytes = lwipMsg.pktSize;
            uint8_t* pPkt = lwipMsg.pPkt;
            
            gWorkPkt++;
            if ( gWorkPkt >= MAX_EEM_POOL)
                gWorkPkt =0;

            logMsg(LogMsg_Debug, "%s %d lwipRX_t", __PRETTY_FUNCTION__, __LINE__);
            
            // fire off another request while processing current pkt.
            uint32_t _regs = taskENTER_CRITICAL_FROM_ISR();
                USBD_EEM_SetRxBuffer(&hUsbDeviceFS, &Rx_Buff[gWorkPkt][0]);
                USBD_EEM_ReceivePacket(&hUsbDeviceFS);
             taskEXIT_CRITICAL_FROM_ISR(_regs);

            do
            {
                if(eemRx.pktLen == 0 && eemRx.p == nullptr)
                {
                    if(remainingBytes < 2)
                    {
                        eemRx.pktLen = 0;
                        eemRx.pktToGo = 0;
                        remainingBytes -= remainingBytes;
                        pPkt += remainingBytes;
                        break;
                    }

                    uint16_t eemHdr = *((uint16_t*)pPkt);
                    pPkt += sizeof(uint16_t);
                    remainingBytes -= sizeof(uint16_t);
                    if(eemHdr & EEM_IS_COMMAND)
                    {
                        if(EEM_CMD_IS_RESERVED & eemHdr)
                        {
                            eemRx.pktLen = 0;
                            eemRx.pktToGo = 0;
                            remainingBytes -= remainingBytes;
                            break;
                        }
                        uint32_t eemCmd = eemHdr >> 11 & EEM_CMD_MASK;
                        switch(eemCmd)
                        {
                        case EEM_CMD_ECHO:
                            break;
                        case EEM_CMD_ECHO_RESPONSE:
                            break;
                        case EEM_CMD_SUSPEND_HINT:
                            break;
                        case EEM_CMD_RESPONSE_HINT:
                            break;
                        case EEM_CMD_RESPONSE_COMPLETE_HINT:
                            break;
                        case EEM_CMD_TICKLE:
                            break;
                        default:
                            break;
                        }
                    }
                    else // is a packet
                    {
                        int lenOfPkt = EEM_PKT_LEN_MASK & eemHdr;

                        if(nullptr == (eemRx.p = pbuf_alloc(PBUF_RAW, lenOfPkt, PBUF_POOL)))
                        {
                            logMsg(LogMsg_Debug, "%s %d pktRx ERROR pbuf_alloc failed", __PRETTY_FUNCTION__, __LINE__);
                            eemRx.pktLen = 0;
                        }
                        else
                        {
                            eemRx.pktLen = lenOfPkt;
                        }
                        
                        eemRx.pktToGo = 0;
                        continue;
                    }
                }
                else
                {
                    int toCopy = std::min(remainingBytes, eemRx.pktLen - eemRx.pktToGo);
                    if(eemRx.p)
                        memcpy( static_cast<uint8_t*>(eemRx.p->payload) + eemRx.pktToGo, pPkt, toCopy);

                    eemRx.pktToGo += toCopy;
                    remainingBytes -= toCopy;
                    pPkt += toCopy;

                    if(eemRx.pktLen == eemRx.pktToGo)
                    {
                        logMsg(LogMsg_Debug, "%s %d RX", __PRETTY_FUNCTION__, __LINE__);

                        if(eemRx.p && netif->input(eemRx.p, netif) != ERR_OK)
                        {
                            logMsg(LogMsg_Debug, "%s %d pktRx err", __PRETTY_FUNCTION__, __LINE__);
                            pbuf_free(eemRx.p);
                        }
                        else
                        {
                          //  if(eemRx.p)
                            //    pbuf_free(eemRx.p);
                         }
                        eemRx.p 	= nullptr;
                        eemRx.pktToGo 	= 0;
                        eemRx.pktLen 	= 0;
                    }
                }
            } while(remainingBytes > 0);
        }
        break;

        case lwipRequest::lwipTxStart_t:
        {
            if(eemTxPkt.pPkt != nullptr)
            {
               logMsg(LogMsg_Debug, "%s %d lwipTxStart_t queue", __PRETTY_FUNCTION__, __LINE__);

                if(false == eemTxRingBuffer.push(lwipMsg.pTxPkt))
                {
                    pbuf_free(lwipMsg.pTxPkt);
                }

                continue;
            }

            if (false == localTx(lwipMsg, eemTxPkt))
            {
                //drop packet
                memset(&eemTxPkt, 0, sizeof(eemTxPkt));
            }
         
        }
        break;

        case lwipRequest::eemUSBTxDone_t:
        {
            logMsg(LogMsg_Debug, "%s %d eemUSBTxDone_t %d ", __PRETTY_FUNCTION__, __LINE__, lwipMsg.pktSize);
            memset(&eemTxPkt, 0, sizeof(eemTxPkt));
            
            //OK 
             while(true == eemTxRingBuffer.pop(lwipMsg.pTxPkt))
             {
                 if (false == localTx(lwipMsg, eemTxPkt))
                 {
                      memset(&eemTxPkt, 0, sizeof(eemTxPkt));  //drop pkt..
                      continue;  // get next packet ..  
                 }
                 else
                 {
                     break; //continue to process tx pkt.. 
                 }
             }
        }
        break;

        case lwipRequest::lwipLink_t:
        {
            if(_gIsUSBLinkActive && !netif_is_up(&gnetif))
            {
                logMsg(LogMsg_Debug, "%s %d update interface ", __PRETTY_FUNCTION__, __LINE__);

                netif_set_link_up(&gnetif);
                netif_set_up(&gnetif);
            }
        }
        break;
        }
    }
}

#if !LWIP_ARP
/**
 * This function has to be completed by user in case of ARP OFF.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if ...
 */
static err_t low_level_output_arp_off(struct netif* netif, struct pbuf* q, ip_addr_t* ipaddr)
{
    err_t errval;
    errval = ERR_OK;

    /* USER CODE BEGIN 5 */

    /* USER CODE END 5 */

    return errval;
}
#endif /* LWIP_ARP */

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif* netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = 'U';
    netif->name[1] = 'S';
/* We directly use etharp_output() here to save a function call.
 * You can instead declare your own function an call etharp_output()
 * from it if you have to do some checks before sending (e.g. if link
 * is available...) */

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
    netif->output = etharp_output;
#else
    /* The user should write ist own code in low_level_output_arp_off function */
    netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */

#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */

    netif->linkoutput = low_level_output;

    /* initialize the hardware */
    low_level_init(netif);

    return ERR_OK;
}

/* USER CODE BEGIN 6 */


/* USER CODE END 6 */

/* USER CODE BEGIN 7 */

/* USER CODE END 7 */

#if LWIP_NETIF_LINK_CALLBACK
/**
  * @brief  Link callback function, this function is called on change of link status
  *         to update low level driver configuration.
* @param  netif: The network interface
  * @retval None
  */
extern "C" void ethernetif_update_config(struct netif* netif)
{

    if(netif_is_link_up(netif))
    {
    }
    else
    {
    }

    ethernetif_notify_conn_changed(netif);
}

/* USER CODE BEGIN 8 */
/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */
__weak void ethernetif_notify_conn_changed(struct netif* netif)
{
    /* NOTE : This is function could be implemented in user file
              when the callback is needed,
    */
}

#endif


extern "C" bool isUSBLinkActive()
{
	return _gIsUSBLinkActive;
}
/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
