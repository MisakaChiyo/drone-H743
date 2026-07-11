#ifndef BSP_RANGEFINDER_H
#define BSP_RANGEFINDER_H

#include "drv_tfmini.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t initialized;
    uint8_t rx_active;
    uint32_t bytes;
    uint32_t frames;
    uint32_t checksum_errors;
    uint32_t frame_errors;
    uint32_t rx_restarts;
    uint32_t dma_events;
    uint32_t dma_last_size;
    uint32_t uart_errors;
    uint32_t last_uart_error;
    DRV_TFMINI_Frame latest;
} BSP_RangefinderStatus;

DRV_TFMINI_Status BSP_Rangefinder_Init(void);
void BSP_Rangefinder_Service(void);
void BSP_Rangefinder_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void BSP_Rangefinder_OnUartError(UART_HandleTypeDef *huart);
void BSP_Rangefinder_GetStatus(BSP_RangefinderStatus *status);

#ifdef __cplusplus
}
#endif

#endif
