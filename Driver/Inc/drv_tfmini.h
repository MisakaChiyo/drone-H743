#ifndef DRV_TFMINI_H
#define DRV_TFMINI_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_TFMINI_BAUD_RATE 115200U
#define DRV_TFMINI_FRAME_LEN  9U

typedef enum {
    DRV_TFMINI_OK = 0,
    DRV_TFMINI_ERROR,
    DRV_TFMINI_INVALID_ARG
} DRV_TFMINI_Status;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t baud_rate;
    uint8_t *rx_dma_buffer;
    uint16_t rx_dma_size;
    void (*cache_invalidate)(const void *addr, uint32_t len);
} DRV_TFMINI_Bus;

typedef struct {
    uint16_t distance_cm;
    uint16_t strength;
    int16_t temperature_raw;
    float temperature_c;
    uint32_t received_ms;
} DRV_TFMINI_Frame;

typedef struct {
    DRV_TFMINI_Bus bus;
    uint8_t initialized;
    uint8_t rx_active;
    uint8_t frame[DRV_TFMINI_FRAME_LEN];
    uint8_t offset;
    uint16_t rx_dma_pos;
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
} DRV_TFMINI_Device;

DRV_TFMINI_Status DRV_TFMINI_Init(DRV_TFMINI_Device *dev,
                                   const DRV_TFMINI_Bus *bus);
void DRV_TFMINI_Service(DRV_TFMINI_Device *dev);
uint8_t DRV_TFMINI_ConsumeByte(DRV_TFMINI_Device *dev, uint8_t byte);
void DRV_TFMINI_OnUartRxEvent(DRV_TFMINI_Device *dev, uint16_t size);
void DRV_TFMINI_OnUartError(DRV_TFMINI_Device *dev, uint32_t error_code);

#ifdef __cplusplus
}
#endif

#endif
