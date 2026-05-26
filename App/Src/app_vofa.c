#include "app_vofa.h"

#include "app_aiwb2.h"
#include "app_messages.h"
#include "app_tasks.h"
#include "app_uart.h"

#include <string.h>

/*
 * VOFA+ JustFloat frame format:
 *   [payload: float32 LE × count] [tail: 0x00 0x00 0x80 0x7f]
 *
 * The tail is the byte representation of -1.1754943508222875e-38,
 * which VOFA+ uses as a frame delimiter.
 *
 * Data is sent through the shared uartTxQueue (DMA) to avoid conflict
 * with the UART task's DMA-based TX.
 */
static const uint8_t app_vofa_tail[4U] = {0x00U, 0x00U, 0x80U, 0x7FU};

void APP_VOFA_SendFloats(const float *data, uint8_t count)
{
    APP_UART_TxMessage tx_msg;
    uint16_t payload_bytes;
    uint16_t total_bytes;

    if ((data == NULL) || (count == 0U) || (count > APP_VOFA_MAX_FLOATS)) {
        return;
    }

    if (APP_AiWB2_IsSocketReady() == 0U) {
        return;
    }

    payload_bytes = (uint16_t)count * 4U;
    total_bytes   = payload_bytes + 4U;

    if (total_bytes > APP_UART_TX_TEXT_SIZE) {
        return;
    }

    memcpy(tx_msg.text, data, payload_bytes);
    memcpy(tx_msg.text + payload_bytes, app_vofa_tail, 4U);
    tx_msg.length   = total_bytes;
    tx_msg.function = APP_UART_TX_FUNCTION_VOFA_SOCKET;

    if (uartTxQueueHandle == 0) {
        return;
    }

    if (osMessageQueuePut(uartTxQueueHandle, &tx_msg, 0U, 0U) != osOK) {
        return;
    }

    APP_UART_NotifyTxPending();
}
