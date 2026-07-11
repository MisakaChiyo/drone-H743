#include "bsp_rangefinder.h"

#include "bsp_board.h"

#include <string.h>

#define BSP_RANGEFINDER_DMA_RX_SIZE 128U

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t rangefinder_dma_rx_buffer[BSP_RANGEFINDER_DMA_RX_SIZE];
static DRV_TFMINI_Device rangefinder_dev;

DRV_TFMINI_Status BSP_Rangefinder_Init(void)
{
    DRV_TFMINI_Bus bus = *BSP_Board_GetRangefinderBus();

    bus.rx_dma_buffer = rangefinder_dma_rx_buffer;
    bus.rx_dma_size = BSP_RANGEFINDER_DMA_RX_SIZE;
    return DRV_TFMINI_Init(&rangefinder_dev, &bus);
}

void BSP_Rangefinder_Service(void)
{
    DRV_TFMINI_Service(&rangefinder_dev);
}

void BSP_Rangefinder_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if ((huart == NULL) || (huart->Instance != UART8)) {
        return;
    }
    DRV_TFMINI_OnUartRxEvent(&rangefinder_dev, size);
}

void BSP_Rangefinder_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART8)) {
        return;
    }
    DRV_TFMINI_OnUartError(&rangefinder_dev, huart->ErrorCode);
}

void BSP_Rangefinder_GetStatus(BSP_RangefinderStatus *status)
{
    if (status == NULL) {
        return;
    }
    memset(status, 0, sizeof(*status));
    status->initialized = rangefinder_dev.initialized;
    status->rx_active = rangefinder_dev.rx_active;
    status->bytes = rangefinder_dev.bytes;
    status->frames = rangefinder_dev.frames;
    status->checksum_errors = rangefinder_dev.checksum_errors;
    status->frame_errors = rangefinder_dev.frame_errors;
    status->rx_restarts = rangefinder_dev.rx_restarts;
    status->dma_events = rangefinder_dev.dma_events;
    status->dma_last_size = rangefinder_dev.dma_last_size;
    status->uart_errors = rangefinder_dev.uart_errors;
    status->last_uart_error = rangefinder_dev.last_uart_error;
    status->latest = rangefinder_dev.latest;
}
