

#pragma once
#ifndef __cplusplus
#include <stdbool.h>
#endif
    
typedef struct 
{
    SPI_HandleTypeDef hspi;
    SemaphoreHandle_t xSemaHandle;


}spiWrapper_t;

#ifdef __cplusplus
extern "C" {
#endif
    
bool addSemaToSPIHandle(int handle, SemaphoreHandle_t semaphore);
int SPITransmitReceive( int handle,	uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,uint32_t waitInTicks);

#ifdef __cplusplus
}
#endif
