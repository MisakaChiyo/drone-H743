#include "app_uart.h"

#include "app_control.h"
#include "app_tasks.h"
#include "bsp_led.h"
#include "bsp_uart.h"

#include "usart.h"

#include <string.h>

#define APP_UART_RX_LINE_SIZE 128U

static char app_uart_rx_line[APP_UART_RX_LINE_SIZE];
static uint16_t app_uart_rx_used;
static uint32_t app_uart_rx_bytes;
static uint32_t app_uart_rx_lines;
static uint32_t app_uart_rx_overflows;
static uint32_t app_uart_rx_errors;
static uint32_t app_uart_last_stats_ms;

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
    ++app_uart_rx_errors;
}

void APP_UART_Task_Init(void)
{
    BSP_LED_Off(LED_1);
    app_uart_rx_used = 0U;
    app_uart_rx_bytes = 0U;
    app_uart_rx_lines = 0U;
    app_uart_rx_overflows = 0U;
    app_uart_rx_errors = 0U;
    app_uart_last_stats_ms = HAL_GetTick();
    APP_Control_Init();
}

static void app_uart_poll_rx(void)
{
    uint8_t byte;
    HAL_StatusTypeDef status;

    app_uart_clear_errors();

    while ((status = HAL_UART_Receive(&huart1, &byte, 1U, 0U)) == HAL_OK) {
        ++app_uart_rx_bytes;
        if ((byte == '\n') || (byte == '\r')) {
            if (app_uart_rx_used > 0U) {
                app_uart_rx_line[app_uart_rx_used] = '\0';
                BSP_LED_On(LED_1);
                APP_Control_ProcessLine(app_uart_rx_line);
                BSP_LED_Off(LED_1);
                app_uart_rx_used = 0U;
                ++app_uart_rx_lines;
            }
            continue;
        }

        if (app_uart_rx_used < (APP_UART_RX_LINE_SIZE - 1U)) {
            app_uart_rx_line[app_uart_rx_used++] = (char)byte;
        } else {
            app_uart_rx_used = 0U;
            ++app_uart_rx_overflows;
        }
    }

    if (status == HAL_ERROR) {
        app_uart_clear_errors();
    }
}

static void app_uart_poll_tx(void)
{
    APP_UART_TxMessage tx_message;

    if (uartTxQueueHandle == 0) {
        return;
    }

    if (osMessageQueueGet(uartTxQueueHandle, &tx_message, 0U, 0U) != osOK) {
        return;
    }

    if (tx_message.length == 0U) {
        return;
    }

    BSP_LED_On(LED_1);
    (void)BSP_UART_Transmit_USART1((const uint8_t *)tx_message.text,
                                   tx_message.length,
                                   100U);
    BSP_LED_Off(LED_1);
}

void APP_UART_Task_Step(void)
{
    uint32_t now_ms;

    app_uart_poll_rx();
    APP_Control_Tick();
    now_ms = HAL_GetTick();
    if ((now_ms - app_uart_last_stats_ms) >= 2000U) {
        app_uart_last_stats_ms = now_ms;
        APP_Control_ReportUartStats(app_uart_rx_bytes,
                                    app_uart_rx_lines,
                                    app_uart_rx_overflows,
                                    app_uart_rx_errors);
    }
    app_uart_poll_tx();
    osDelay(2U);
}
