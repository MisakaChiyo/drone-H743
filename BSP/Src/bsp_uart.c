#include "bsp_uart.h"

#include "usart.h"

void BSP_UART_Release_USART1_ForExternalDebug(void)
{
#if (BSP_UART_USART1_OUTPUT_ENABLED == 0U)
    (void)HAL_UART_DeInit(&huart1);
#endif
}

HAL_StatusTypeDef BSP_UART_Transmit_USART1(const uint8_t *data,
                                           uint16_t length,
                                           uint32_t timeout_ms)
{
#if (BSP_UART_USART1_OUTPUT_ENABLED == 0U)
    (void)data;
    (void)length;
    (void)timeout_ms;
    return HAL_OK;
#else
    if ((data == 0) || (length == 0U)) {
        return HAL_OK;
    }

    return HAL_UART_Transmit(&huart1, (uint8_t *)data, length, timeout_ms);
#endif
}
