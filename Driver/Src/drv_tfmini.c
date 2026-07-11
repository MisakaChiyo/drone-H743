#include "drv_tfmini.h"

#include <string.h>

static uint16_t tfmini_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static void tfmini_restart_rx(DRV_TFMINI_Device *dev)
{
    HAL_StatusTypeDef status;

    if ((dev == NULL) || (dev->bus.huart == NULL) ||
        (dev->bus.huart->hdmarx == NULL) ||
        (dev->bus.rx_dma_buffer == NULL) || (dev->bus.rx_dma_size == 0U)) {
        return;
    }

    if (dev->bus.huart->RxState != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortReceive(dev->bus.huart);
    }
    dev->rx_dma_pos = 0U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(dev->bus.huart,
                                          dev->bus.rx_dma_buffer,
                                          dev->bus.rx_dma_size);
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(dev->bus.huart->hdmarx, DMA_IT_HT);
        dev->rx_active = 1U;
        dev->rx_restarts++;
    } else {
        dev->rx_active = 0U;
        dev->uart_errors++;
        dev->last_uart_error = dev->bus.huart->ErrorCode;
    }
}

static uint16_t tfmini_dma_write_pos(DRV_TFMINI_Device *dev,
                                     uint16_t event_size)
{
    uint32_t remaining;

    if ((dev == NULL) || (dev->bus.huart == NULL) ||
        (dev->bus.huart->hdmarx == NULL) || (dev->bus.rx_dma_size == 0U)) {
        return 0U;
    }

    remaining = __HAL_DMA_GET_COUNTER(dev->bus.huart->hdmarx);
    if (remaining <= dev->bus.rx_dma_size) {
        return (uint16_t)(dev->bus.rx_dma_size - remaining);
    }
    return (event_size <= dev->bus.rx_dma_size) ? event_size : 0U;
}

static void tfmini_consume_dma(DRV_TFMINI_Device *dev, uint16_t event_size)
{
    uint16_t write_pos;

    if ((dev == NULL) || (dev->initialized == 0U) ||
        (dev->bus.rx_dma_buffer == NULL) || (dev->bus.rx_dma_size == 0U)) {
        return;
    }

    if (dev->bus.cache_invalidate != NULL) {
        dev->bus.cache_invalidate(dev->bus.rx_dma_buffer,
                                  dev->bus.rx_dma_size);
    }
    write_pos = tfmini_dma_write_pos(dev, event_size);
    if (write_pos >= dev->bus.rx_dma_size) {
        write_pos = 0U;
    }
    while (dev->rx_dma_pos != write_pos) {
        (void)DRV_TFMINI_ConsumeByte(dev,
                                     dev->bus.rx_dma_buffer[dev->rx_dma_pos]);
        dev->rx_dma_pos++;
        if (dev->rx_dma_pos >= dev->bus.rx_dma_size) {
            dev->rx_dma_pos = 0U;
        }
    }
}

uint8_t DRV_TFMINI_ConsumeByte(DRV_TFMINI_Device *dev, uint8_t byte)
{
    uint8_t checksum = 0U;

    if (dev == NULL) {
        return 0U;
    }
    dev->bytes++;

    if (dev->offset == 0U) {
        if (byte == 0x59U) {
            dev->frame[dev->offset++] = byte;
        }
        return 0U;
    }
    if (dev->offset == 1U) {
        if (byte == 0x59U) {
            dev->frame[dev->offset++] = byte;
        } else {
            dev->offset = 0U;
            dev->frame_errors++;
        }
        return 0U;
    }

    dev->frame[dev->offset++] = byte;
    if (dev->offset < DRV_TFMINI_FRAME_LEN) {
        return 0U;
    }
    dev->offset = 0U;

    for (uint32_t i = 0U; i < (DRV_TFMINI_FRAME_LEN - 1U); ++i) {
        checksum = (uint8_t)(checksum + dev->frame[i]);
    }
    if (checksum != dev->frame[8]) {
        dev->checksum_errors++;
        if (byte == 0x59U) {
            dev->frame[dev->offset++] = byte;
        }
        return 0U;
    }

    dev->latest.distance_cm = tfmini_u16_le(&dev->frame[2]);
    dev->latest.strength = tfmini_u16_le(&dev->frame[4]);
    dev->latest.temperature_raw = (int16_t)tfmini_u16_le(&dev->frame[6]);
    dev->latest.temperature_c =
        (float)dev->latest.temperature_raw / 8.0f - 256.0f;
    dev->latest.received_ms = HAL_GetTick();
    dev->frames++;
    return 1U;
}

DRV_TFMINI_Status DRV_TFMINI_Init(DRV_TFMINI_Device *dev,
                                   const DRV_TFMINI_Bus *bus)
{
    static const uint8_t set_centimeter_unit[] = {
        0x5AU, 0x05U, 0x05U, 0x01U, 0x65U
    };
    UART_HandleTypeDef *huart;

    if ((dev == NULL) || (bus == NULL) || (bus->huart == NULL) ||
        (bus->rx_dma_buffer == NULL) || (bus->rx_dma_size == 0U)) {
        return DRV_TFMINI_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;
    huart = dev->bus.huart;
    if (dev->bus.baud_rate == 0U) {
        dev->bus.baud_rate = DRV_TFMINI_BAUD_RATE;
    }
    if ((huart->Init.BaudRate != dev->bus.baud_rate) ||
        (huart->Init.WordLength != UART_WORDLENGTH_8B) ||
        (huart->Init.StopBits != UART_STOPBITS_1) ||
        (huart->Init.Parity != UART_PARITY_NONE)) {
        return DRV_TFMINI_ERROR;
    }

    (void)HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                 UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    if (HAL_UART_Transmit(huart,
                          (uint8_t *)set_centimeter_unit,
                          (uint16_t)sizeof(set_centimeter_unit),
                          20U) != HAL_OK) {
        dev->uart_errors++;
        dev->last_uart_error = huart->ErrorCode;
        return DRV_TFMINI_ERROR;
    }
    dev->initialized = 1U;
    tfmini_restart_rx(dev);
    return (dev->rx_active != 0U) ? DRV_TFMINI_OK : DRV_TFMINI_ERROR;
}

void DRV_TFMINI_Service(DRV_TFMINI_Device *dev)
{
    if ((dev == NULL) || (dev->initialized == 0U)) {
        return;
    }
    tfmini_consume_dma(dev, 0U);
    if ((dev->bus.huart->RxState != HAL_UART_STATE_BUSY_RX) ||
        (dev->bus.huart->hdmarx == NULL) ||
        ((((DMA_Stream_TypeDef *)dev->bus.huart->hdmarx->Instance)->CR &
          DMA_SxCR_EN) == 0U)) {
        dev->rx_active = 0U;
        tfmini_restart_rx(dev);
    }
}

void DRV_TFMINI_OnUartRxEvent(DRV_TFMINI_Device *dev, uint16_t size)
{
    if ((dev == NULL) || (dev->initialized == 0U)) {
        return;
    }
    dev->dma_events++;
    dev->dma_last_size = size;
    tfmini_consume_dma(dev, size);
}

void DRV_TFMINI_OnUartError(DRV_TFMINI_Device *dev, uint32_t error_code)
{
    UART_HandleTypeDef *huart;

    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return;
    }
    huart = dev->bus.huart;
    dev->uart_errors++;
    dev->last_uart_error = error_code;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                 UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_AbortReceive(huart);
    dev->rx_active = 0U;
    dev->offset = 0U;
}
