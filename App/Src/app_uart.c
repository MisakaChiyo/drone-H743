#include "app_uart.h"

#include "app_aiwb2.h"
#include "app_control.h"
#include "app_tasks.h"
#include "bsp_led.h"
#include "bsp_uart.h"

#include "usart.h"

#include <stdio.h>
#include <string.h>

#define APP_UART_RX_LINE_SIZE 128U
#define APP_UART_DMA_RX_SIZE  256U
#define APP_UART_TX_LED_PULSE_MS 80U
#define APP_UART_RX_IDLE_LINE_MS 60U
#define APP_UART_DEBUG_LINE_LIMIT 8U

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t app_uart_dma_rx_buffer[APP_UART_DMA_RX_SIZE];

static char app_uart_rx_line[APP_UART_RX_LINE_SIZE];
static uint16_t app_uart_rx_used;
static uint16_t app_uart_dma_rx_pos;
static uint32_t app_uart_rx_bytes;
static uint32_t app_uart_rx_lines;
static uint32_t app_uart_rx_idle_lines;
static uint32_t app_uart_rx_overflows;
static uint32_t app_uart_rx_errors;
static uint32_t app_uart_last_stats_ms;
static uint32_t app_uart_last_rx_byte_ms;
static uint32_t app_uart_tx_led_until_ms;
static uint32_t app_uart_last_tx_count;
static uint32_t app_uart_debug_lines;
static uint8_t app_uart_control_initialized;
static uint8_t app_uart_dma_started;

static uint8_t app_uart_time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

static void app_uart_start_rx_dma(void)
{
    HAL_StatusTypeDef status;

    if (huart1.hdmarx == 0) {
        ++app_uart_rx_errors;
        return;
    }

    app_uart_dma_rx_pos = 0U;
    memset(app_uart_dma_rx_buffer, 0, sizeof(app_uart_dma_rx_buffer));
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                          app_uart_dma_rx_buffer,
                                          APP_UART_DMA_RX_SIZE);
    if (status != HAL_OK) {
        ++app_uart_rx_errors;
        app_uart_dma_started = 0U;
        return;
    }

    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    app_uart_dma_started = 1U;
}

static char *app_uart_normalize_line(char *line, uint16_t length)
{
    uint16_t start_index = 0U;
    uint16_t end_index = length;
    uint16_t out_index = 0U;

    while ((start_index < end_index) &&
           ((uint8_t)line[start_index] <= (uint8_t)' ')) {
        ++start_index;
    }

    if ((start_index < end_index) && (line[start_index] == '>')) {
        ++start_index;
        while ((start_index < end_index) &&
               ((uint8_t)line[start_index] <= (uint8_t)' ')) {
            ++start_index;
        }
    }

    while ((end_index > start_index) &&
           ((uint8_t)line[end_index - 1U] <= (uint8_t)' ')) {
        --end_index;
    }

    while (start_index < end_index) {
        line[out_index++] = line[start_index++];
    }
    line[out_index] = '\0';

    return line;
}

static void app_uart_report_line_debug(const uint8_t *data, uint16_t length)
{
    char text[160];
    uint32_t offset = 0U;
    uint16_t shown = length;
    int written;

    if ((app_uart_control_initialized != 0U) ||
        (app_uart_debug_lines >= APP_UART_DEBUG_LINE_LIMIT)) {
        return;
    }

    if (shown > 16U) {
        shown = 16U;
    }

    written = snprintf(text,
                       sizeof(text),
                       "BOOT uart_line len=%u hex=",
                       (unsigned int)length);
    if (written <= 0) {
        return;
    }
    offset = (uint32_t)written;

    for (uint16_t index = 0U; index < shown; ++index) {
        written = snprintf(&text[offset],
                           sizeof(text) - offset,
                           "%02X%s",
                           (unsigned int)data[index],
                           ((index + 1U) < shown) ? " " : "");
        if (written <= 0) {
            return;
        }
        offset += (uint32_t)written;
        if (offset >= (sizeof(text) - 4U)) {
            break;
        }
    }

    if (shown < length) {
        written = snprintf(&text[offset], sizeof(text) - offset, " ...");
        if (written > 0) {
            offset += (uint32_t)written;
        }
    }

    if (offset < (sizeof(text) - 3U)) {
        text[offset++] = '\r';
        text[offset++] = '\n';
        text[offset] = '\0';
    }

    (void)BSP_UART_Transmit_USART1((const uint8_t *)text,
                                   (uint16_t)offset,
                                   100U);
    ++app_uart_debug_lines;
}

static void app_uart_clear_errors(void)
{
    uint32_t error = HAL_UART_GetError(&huart1);

    if (error == HAL_UART_ERROR_NONE) {
        return;
    }

    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                   UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart1.ErrorCode = HAL_UART_ERROR_NONE;
    app_uart_rx_used = 0U;
    app_uart_dma_started = 0U;
    (void)HAL_UART_AbortReceive(&huart1);
    ++app_uart_rx_errors;
}

void APP_UART_Task_Init(void)
{
    static const char boot_text[] = "BOOT uart_task_init\r\n";

    BSP_LED_Off(LED_1);
    app_uart_rx_used = 0U;
    app_uart_rx_bytes = 0U;
    app_uart_rx_lines = 0U;
    app_uart_rx_idle_lines = 0U;
    app_uart_rx_overflows = 0U;
    app_uart_rx_errors = 0U;
    app_uart_last_stats_ms = HAL_GetTick();
    app_uart_last_rx_byte_ms = app_uart_last_stats_ms;
    app_uart_tx_led_until_ms = app_uart_last_stats_ms;
    app_uart_last_tx_count = BSP_UART_GetUSART1TxCount();
    app_uart_debug_lines = 0U;
    app_uart_control_initialized = 0U;
    app_uart_dma_started = 0U;
    (void)BSP_UART_Transmit_USART1((const uint8_t *)boot_text,
                                   (uint16_t)(sizeof(boot_text) - 1U),
                                   100U);
    APP_AiWB2_Init();
    app_uart_start_rx_dma();
}

static void app_uart_ensure_control_ready(void)
{
    static const char init_begin_text[] = "BOOT control_init_begin\r\n";
    static const char init_done_text[] = "BOOT control_init_done\r\n";

    if ((app_uart_control_initialized != 0U) ||
        (APP_AiWB2_IsTransparent() == 0U)) {
        return;
    }

    (void)BSP_UART_Transmit_USART1((const uint8_t *)init_begin_text,
                                   (uint16_t)(sizeof(init_begin_text) - 1U),
                                   100U);
    APP_Control_Init();
    (void)BSP_UART_Transmit_USART1((const uint8_t *)init_done_text,
                                   (uint16_t)(sizeof(init_done_text) - 1U),
                                   100U);
    app_uart_control_initialized = 1U;
}

static void app_uart_handle_line(char *line, uint16_t length)
{
    char *normalized;

    app_uart_report_line_debug((const uint8_t *)line, length);
    normalized = app_uart_normalize_line(line, length);
    if (*normalized == '\0') {
        return;
    }

    if (APP_AiWB2_IsTransparent() == 0U) {
        if (APP_AiWB2_IsControlPayload(normalized) != 0U) {
            static const char payload_text[] = "BOOT control_payload\r\n";

            (void)BSP_UART_Transmit_USART1((const uint8_t *)payload_text,
                                           (uint16_t)(sizeof(payload_text) - 1U),
                                           100U);
            APP_AiWB2_AssumeTransparent();
            app_uart_ensure_control_ready();
            if (app_uart_control_initialized != 0U) {
                APP_Control_ProcessLine(normalized);
            }
            return;
        }

        APP_AiWB2_ProcessLine(normalized);
        return;
    }

    if (APP_AiWB2_ShouldConsumeTransparentLine(normalized) != 0U) {
        APP_AiWB2_ProcessLine(normalized);
        return;
    }

    if (app_uart_control_initialized != 0U) {
        APP_Control_ProcessLine(normalized);
    }
}

static void app_uart_process_rx_byte(uint8_t byte)
{
    app_uart_last_rx_byte_ms = HAL_GetTick();
    ++app_uart_rx_bytes;
    if ((byte == '\n') || (byte == '\r')) {
        if (app_uart_rx_used > 0U) {
            app_uart_rx_line[app_uart_rx_used] = '\0';
            app_uart_handle_line(app_uart_rx_line, app_uart_rx_used);
            app_uart_rx_used = 0U;
            ++app_uart_rx_lines;
        }
        return;
    }

    if (app_uart_rx_used < (APP_UART_RX_LINE_SIZE - 1U)) {
        app_uart_rx_line[app_uart_rx_used++] = (char)byte;
    } else {
        app_uart_rx_used = 0U;
        ++app_uart_rx_overflows;
    }
}

static void app_uart_flush_idle_line(uint32_t now_ms)
{
    if (app_uart_rx_used == 0U) {
        return;
    }

    if (app_uart_time_reached(now_ms,
                              app_uart_last_rx_byte_ms + APP_UART_RX_IDLE_LINE_MS) == 0U) {
        return;
    }

    app_uart_rx_line[app_uart_rx_used] = '\0';
    app_uart_handle_line(app_uart_rx_line, app_uart_rx_used);
    app_uart_rx_used = 0U;
    ++app_uart_rx_lines;
    ++app_uart_rx_idle_lines;
}

static uint16_t app_uart_dma_write_pos(void)
{
    uint32_t remaining;

    if ((huart1.hdmarx == 0) || (app_uart_dma_started == 0U)) {
        return app_uart_dma_rx_pos;
    }

    remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (remaining > APP_UART_DMA_RX_SIZE) {
        return app_uart_dma_rx_pos;
    }

    return (uint16_t)(APP_UART_DMA_RX_SIZE - remaining);
}

static void app_uart_poll_rx(void)
{
    uint16_t write_pos;

    app_uart_clear_errors();
    if (app_uart_dma_started == 0U) {
        app_uart_start_rx_dma();
    }

    write_pos = app_uart_dma_write_pos();
    while (app_uart_dma_rx_pos != write_pos) {
        app_uart_process_rx_byte(app_uart_dma_rx_buffer[app_uart_dma_rx_pos]);
        ++app_uart_dma_rx_pos;
        if (app_uart_dma_rx_pos >= APP_UART_DMA_RX_SIZE) {
            app_uart_dma_rx_pos = 0U;
        }
    }
}

static void app_uart_poll_tx(void)
{
    APP_UART_TxMessage tx_message;

    if (uartTxQueueHandle == 0) {
        return;
    }

    if ((APP_AiWB2_IsTransparent() == 0U) ||
        (app_uart_control_initialized == 0U)) {
        return;
    }

    if (osMessageQueueGet(uartTxQueueHandle, &tx_message, 0U, 0U) != osOK) {
        return;
    }

    if (tx_message.length == 0U) {
        return;
    }

    (void)BSP_UART_Transmit_USART1((const uint8_t *)tx_message.text,
                                   tx_message.length,
                                   100U);
}

static void app_uart_update_tx_led(uint32_t now_ms)
{
    uint32_t tx_count = BSP_UART_GetUSART1TxCount();

    if (tx_count != app_uart_last_tx_count) {
        app_uart_last_tx_count = tx_count;
        app_uart_tx_led_until_ms = now_ms + APP_UART_TX_LED_PULSE_MS;
        BSP_LED_On(LED_1);
        return;
    }

    if (app_uart_time_reached(now_ms, app_uart_tx_led_until_ms) != 0U) {
        BSP_LED_Off(LED_1);
    }
}

void APP_UART_Task_Step(void)
{
    uint32_t now_ms;

    app_uart_poll_rx();
    now_ms = HAL_GetTick();
    app_uart_flush_idle_line(now_ms);
    APP_AiWB2_Tick();
    app_uart_ensure_control_ready();
    if ((app_uart_control_initialized != 0U) &&
        (APP_AiWB2_IsTransparent() != 0U)) {
        APP_Control_Tick();
    }
    if ((app_uart_control_initialized != 0U) &&
        (APP_AiWB2_IsTransparent() != 0U) &&
        ((now_ms - app_uart_last_stats_ms) >= 2000U)) {
        app_uart_last_stats_ms = now_ms;
        APP_Control_ReportUartStats(app_uart_rx_bytes,
                                    app_uart_rx_lines,
                                    app_uart_rx_overflows,
                                    app_uart_rx_errors);
    } else if ((app_uart_control_initialized == 0U) &&
               ((now_ms - app_uart_last_stats_ms) >= 2000U)) {
        char stats_text[96];
        int written;

        app_uart_last_stats_ms = now_ms;
        written = snprintf(stats_text,
                           sizeof(stats_text),
                           "BOOT uart_wait_control rx_bytes=%lu rx_lines=%lu rx_idle=%lu rx_overflows=%lu rx_errors=%lu\r\n",
                           (unsigned long)app_uart_rx_bytes,
                           (unsigned long)app_uart_rx_lines,
                           (unsigned long)app_uart_rx_idle_lines,
                           (unsigned long)app_uart_rx_overflows,
                           (unsigned long)app_uart_rx_errors);
        if (written > 0) {
            uint16_t length = (uint16_t)written;

            if ((uint32_t)written >= sizeof(stats_text)) {
                length = (uint16_t)(sizeof(stats_text) - 1U);
            }
            (void)BSP_UART_Transmit_USART1((const uint8_t *)stats_text,
                                           length,
                                           100U);
        }
    }
    app_uart_poll_tx();
    app_uart_update_tx_led(HAL_GetTick());
    osDelay(2U);
}
