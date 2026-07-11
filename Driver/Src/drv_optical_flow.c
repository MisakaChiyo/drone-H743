#include "drv_optical_flow.h"

#include <string.h>

#define LC307_SENSOR_IIC_ADDR      0xDCU
#define LC307_CONFIG_DELAY_MS      100U
#define LC307_CONFIG_TIMEOUT_MS    100U
#define LC307_CONFIG_AB_RETRIES    3U
#define LC307_CONFIG_BB_TIMEOUT_MS 50U
#define LC307_CONFIG_BB_RETRIES    5U
#define LC307_CONFIG_BB_GAP_MS     5U
#define LC307_CONFIG_RESET_MS      50U
#define LC307_STATUS_SCAN_MAX      32U
#define LC307_STATUS_BYTE_TIMEOUT_MS 5U
#define LC307_DRAIN_BYTE_MAX       96U
#define LC307_DRAIN_TIMEOUT_MS     2U
#define LC307_STREAM_PROBE_MS      500U
#define LC307_STREAM_PROBE_FRAMES  2U
#define LC307_CMD_AA               0xAAU
#define LC307_CMD_AB               0xABU
#define LC307_CMD_BB               0xBBU
#define LC307_CMD_DD               0xDDU
#define LC307_ERR_AB_TX            1U
#define LC307_ERR_AB_RX            2U
#define LC307_ERR_BB_TX            3U
#define LC307_ERR_BB_RX            4U
#define LC307_ERR_DD_TX            5U

typedef struct {
    uint8_t reg;
    uint8_t value;
} LC307_ConfigEntry;

/*
 * LC307 requires the repeated 0xBB sensor register phase before it outputs
 * FE 0A frames. The PDF references a matching config file; the available
 * reference data starts with the same 0x12/0x80 pair shown in the LC307 PDF.
 */
static const LC307_ConfigEntry lc307_config_table[] = {
    {0x12U, 0x80U}, {0x11U, 0x30U}, {0x1BU, 0x06U}, {0x6BU, 0x43U},
    {0x12U, 0x20U}, {0x3AU, 0x00U}, {0x15U, 0x02U}, {0x62U, 0x81U},
    {0x08U, 0xA0U}, {0x06U, 0x68U}, {0x2BU, 0x20U}, {0x92U, 0x25U},
    {0x27U, 0x97U}, {0x17U, 0x01U}, {0x18U, 0x79U}, {0x19U, 0x00U},
    {0x1AU, 0xA0U}, {0x03U, 0x00U}, {0x13U, 0x00U}, {0x01U, 0x13U},
    {0x02U, 0x20U}, {0x87U, 0x16U}, {0x8CU, 0x01U}, {0x8DU, 0xCCU},
    {0x13U, 0x07U}, {0x33U, 0x10U}, {0x34U, 0x1DU}, {0x35U, 0x46U},
    {0x36U, 0x40U}, {0x37U, 0xA4U}, {0x38U, 0x7CU}, {0x65U, 0x46U},
    {0x66U, 0x46U}, {0x6EU, 0x20U}, {0x9BU, 0xA4U}, {0x9CU, 0x7CU},
    {0xBCU, 0x0CU}, {0xBDU, 0xA4U}, {0xBEU, 0x7CU}, {0x20U, 0x09U},
    {0x09U, 0x03U}, {0x72U, 0x2FU}, {0x73U, 0x2FU}, {0x74U, 0xA7U},
    {0x75U, 0x12U}, {0x79U, 0x8DU}, {0x7AU, 0x00U}, {0x7EU, 0xFAU},
    {0x70U, 0x0FU}, {0x7CU, 0x84U}, {0x7DU, 0xBAU}, {0x5BU, 0xC2U},
    {0x76U, 0x90U}, {0x7BU, 0x55U}, {0x71U, 0x46U}, {0x77U, 0xDDU},
    {0x13U, 0x0FU}, {0x8AU, 0x10U}, {0x8BU, 0x20U}, {0x8EU, 0x21U},
    {0x8FU, 0x40U}, {0x94U, 0x41U}, {0x95U, 0x7EU}, {0x96U, 0x7FU},
    {0x97U, 0xF3U}, {0x13U, 0x07U}, {0x24U, 0x58U}, {0x97U, 0x48U},
    {0x25U, 0x08U}, {0x94U, 0xB5U}, {0x95U, 0xC0U}, {0x80U, 0xF4U},
    {0x81U, 0xE0U}, {0x82U, 0x1BU}, {0x83U, 0x37U}, {0x84U, 0x39U},
    {0x85U, 0x58U}, {0x86U, 0xFFU}, {0x89U, 0x15U}, {0x8AU, 0xB8U},
    {0x8BU, 0x99U}, {0x39U, 0x98U}, {0x3FU, 0x98U}, {0x90U, 0xA0U},
    {0x91U, 0xE0U}, {0x40U, 0x20U}, {0x41U, 0x28U}, {0x42U, 0x26U},
    {0x43U, 0x25U}, {0x44U, 0x1FU}, {0x45U, 0x1AU}, {0x46U, 0x16U},
    {0x47U, 0x12U}, {0x48U, 0x0FU}, {0x49U, 0x0DU}, {0x4BU, 0x0BU},
    {0x4CU, 0x0AU}, {0x4EU, 0x08U}, {0x4FU, 0x06U}, {0x50U, 0x06U},
    {0x5AU, 0x56U}, {0x51U, 0x1BU}, {0x52U, 0x04U}, {0x53U, 0x4AU},
    {0x54U, 0x26U}, {0x57U, 0x75U}, {0x58U, 0x2BU}, {0x5AU, 0xD6U},
    {0x51U, 0x28U}, {0x52U, 0x1EU}, {0x53U, 0x9EU}, {0x54U, 0x70U},
    {0x57U, 0x50U}, {0x58U, 0x07U}, {0x5CU, 0x28U}, {0xB0U, 0xE0U},
    {0xB1U, 0xC0U}, {0xB2U, 0xB0U}, {0xB3U, 0x4FU}, {0xB4U, 0x63U},
    {0xB4U, 0xE3U}, {0xB1U, 0xF0U}, {0xB2U, 0xA0U}, {0x55U, 0x00U},
    {0x56U, 0x40U}, {0x96U, 0x50U}, {0x9AU, 0x30U}, {0x6AU, 0x81U},
    {0x23U, 0x33U}, {0xA0U, 0xD0U}, {0xA1U, 0x31U}, {0xA6U, 0x04U},
    {0xA2U, 0x0FU}, {0xA3U, 0x2BU}, {0xA4U, 0x0FU}, {0xA5U, 0x2BU},
    {0xA7U, 0x9AU}, {0xA8U, 0x1CU}, {0xA9U, 0x11U}, {0xAAU, 0x16U},
    {0xABU, 0x16U}, {0xACU, 0x3CU}, {0xADU, 0xF0U}, {0xAEU, 0x57U},
    {0xC6U, 0xAAU}, {0xD2U, 0x78U}, {0xD0U, 0xB4U}, {0xD1U, 0x00U},
    {0xC8U, 0x10U}, {0xC9U, 0x12U}, {0xD3U, 0x09U}, {0xD4U, 0x2AU},
    {0xEEU, 0x4CU}, {0x7EU, 0xFAU}, {0x74U, 0xA7U}, {0x78U, 0x4EU},
    {0x60U, 0xE7U}, {0x61U, 0xC8U}, {0x6DU, 0x70U}, {0x1EU, 0x39U},
    {0x98U, 0x1AU}
};

static uint16_t flow_get_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static int16_t flow_i32_to_i16_sat(int32_t value)
{
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return (int16_t)value;
}

static uint16_t flow_u32_to_u16_sat(uint32_t value)
{
    if (value > 65535UL) {
        return 65535U;
    }
    return (uint16_t)value;
}

static void flow_update_raw_stats(DRV_OPTICAL_FLOW_Device *dev,
                                  const DRV_OPTICAL_FLOW_Frame *frame)
{
    int32_t fx_sum = 0L;
    int32_t fy_sum = 0L;
    uint32_t dt_sum = 0UL;
    uint32_t ground_sum = 0UL;
    int16_t fx_min;
    int16_t fx_max;
    int16_t fy_min;
    int16_t fy_max;
    uint16_t dt_min;
    uint16_t dt_max;
    uint16_t ground_min;
    uint16_t ground_max;
    uint16_t count;

    if ((dev == NULL) || (frame == NULL)) {
        return;
    }

    dev->raw_window[dev->raw_window_pos] = *frame;
    dev->raw_window_pos++;
    if (dev->raw_window_pos >= DRV_OPTICAL_FLOW_RAW_WINDOW) {
        dev->raw_window_pos = 0U;
    }
    if (dev->raw_window_count < DRV_OPTICAL_FLOW_RAW_WINDOW) {
        dev->raw_window_count++;
    }

    count = dev->raw_window_count;
    fx_min = dev->raw_window[0].flow_x_integral;
    fx_max = fx_min;
    fy_min = dev->raw_window[0].flow_y_integral;
    fy_max = fy_min;
    dt_min = dev->raw_window[0].integration_timespan_us;
    dt_max = dt_min;
    ground_min = dev->raw_window[0].ground_distance;
    ground_max = ground_min;

    for (uint16_t i = 0U; i < count; ++i) {
        const DRV_OPTICAL_FLOW_Frame *sample = &dev->raw_window[i];
        fx_sum += sample->flow_x_integral;
        fy_sum += sample->flow_y_integral;
        dt_sum += sample->integration_timespan_us;
        ground_sum += sample->ground_distance;

        if (sample->flow_x_integral < fx_min) { fx_min = sample->flow_x_integral; }
        if (sample->flow_x_integral > fx_max) { fx_max = sample->flow_x_integral; }
        if (sample->flow_y_integral < fy_min) { fy_min = sample->flow_y_integral; }
        if (sample->flow_y_integral > fy_max) { fy_max = sample->flow_y_integral; }
        if (sample->integration_timespan_us < dt_min) { dt_min = sample->integration_timespan_us; }
        if (sample->integration_timespan_us > dt_max) { dt_max = sample->integration_timespan_us; }
        if (sample->ground_distance < ground_min) { ground_min = sample->ground_distance; }
        if (sample->ground_distance > ground_max) { ground_max = sample->ground_distance; }
    }

    dev->raw_stats.count = count;
    dev->raw_stats.flow_x_mean = flow_i32_to_i16_sat(fx_sum / (int32_t)count);
    dev->raw_stats.flow_y_mean = flow_i32_to_i16_sat(fy_sum / (int32_t)count);
    dev->raw_stats.integration_timespan_mean_us =
        flow_u32_to_u16_sat(dt_sum / (uint32_t)count);
    dev->raw_stats.ground_distance_mean =
        flow_u32_to_u16_sat(ground_sum / (uint32_t)count);
    dev->raw_stats.flow_x_peak_to_peak =
        flow_i32_to_i16_sat((int32_t)fx_max - (int32_t)fx_min);
    dev->raw_stats.flow_y_peak_to_peak =
        flow_i32_to_i16_sat((int32_t)fy_max - (int32_t)fy_min);
    dev->raw_stats.integration_timespan_peak_to_peak_us =
        (uint16_t)(dt_max - dt_min);
    dev->raw_stats.ground_distance_peak_to_peak =
        (uint16_t)(ground_max - ground_min);
}

static uint32_t flow_timeout_ms(const DRV_OPTICAL_FLOW_Device *dev)
{
    if ((dev != NULL) && (dev->bus.timeout_ms != 0U)) {
        return dev->bus.timeout_ms;
    }
    return LC307_CONFIG_TIMEOUT_MS;
}

static void flow_delay_ms(DRV_OPTICAL_FLOW_Device *dev, uint32_t delay_ms)
{
    if ((dev != NULL) && (dev->bus.delay_ms != NULL)) {
        dev->bus.delay_ms(delay_ms);
        return;
    }
    HAL_Delay(delay_ms);
}

static void flow_reset_parser(DRV_OPTICAL_FLOW_Device *dev)
{
    dev->offset = 0U;
}

static void flow_restart_rx(DRV_OPTICAL_FLOW_Device *dev)
{
    HAL_StatusTypeDef status;

    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return;
    }

    if ((dev->bus.rx_dma_buffer != NULL) && (dev->bus.rx_dma_size != 0U) &&
        (dev->bus.huart->hdmarx != NULL)) {
        dev->rx_dma_pos = 0U;
        status = HAL_UARTEx_ReceiveToIdle_DMA(dev->bus.huart,
                                              dev->bus.rx_dma_buffer,
                                              dev->bus.rx_dma_size);
        if (status == HAL_OK) {
            __HAL_DMA_DISABLE_IT(dev->bus.huart->hdmarx, DMA_IT_HT);
        }
    } else {
        status = HAL_UART_Receive_IT(dev->bus.huart, &dev->rx_byte, 1U);
    }

    if (status == HAL_OK) {
        dev->rx_active = 1U;
        dev->rx_restarts++;
    } else {
        dev->rx_active = 0U;
        dev->uart_errors++;
        dev->last_uart_error = dev->bus.huart->ErrorCode;
    }
}

static uint16_t flow_dma_write_pos(DRV_OPTICAL_FLOW_Device *dev,
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

    if (event_size <= dev->bus.rx_dma_size) {
        return event_size;
    }

    return dev->rx_dma_pos;
}

static void flow_abort_rx(DRV_OPTICAL_FLOW_Device *dev)
{
    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return;
    }

    (void)HAL_UART_AbortReceive_IT(dev->bus.huart);
    if (dev->bus.huart->hdmarx != NULL) {
        (void)HAL_DMA_Abort(dev->bus.huart->hdmarx);
    }
    dev->rx_active = 0U;
}

static void flow_lc307_drain_rx(DRV_OPTICAL_FLOW_Device *dev)
{
    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return;
    }

    for (uint32_t index = 0U; index < LC307_DRAIN_BYTE_MAX; ++index) {
        uint8_t byte = 0U;
        HAL_StatusTypeDef status =
            HAL_UART_Receive(dev->bus.huart, &byte, 1U, LC307_DRAIN_TIMEOUT_MS);
        dev->config_last_hal_status = (uint32_t)status;
        if (status != HAL_OK) {
            dev->bus.huart->ErrorCode = HAL_UART_ERROR_NONE;
            break;
        }
    }
}

static uint8_t flow_lc307_probe_stream(DRV_OPTICAL_FLOW_Device *dev,
                                       uint32_t probe_ms,
                                       uint32_t min_frames)
{
    uint32_t start_ms;
    uint32_t start_frames;

    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return 0U;
    }

    flow_reset_parser(dev);
    start_ms = HAL_GetTick();
    start_frames = dev->frames;

    while ((HAL_GetTick() - start_ms) < probe_ms) {
        uint8_t byte = 0U;
        HAL_StatusTypeDef status =
            HAL_UART_Receive(dev->bus.huart,
                             &byte,
                             1U,
                             LC307_STATUS_BYTE_TIMEOUT_MS);

        dev->config_last_hal_status = (uint32_t)status;
        if (status == HAL_TIMEOUT) {
            continue;
        }
        if (status != HAL_OK) {
            dev->last_uart_error = dev->bus.huart->ErrorCode;
            __HAL_UART_CLEAR_FLAG(dev->bus.huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                             UART_CLEAR_PEF | UART_CLEAR_FEF);
            dev->bus.huart->ErrorCode = HAL_UART_ERROR_NONE;
            continue;
        }

        (void)DRV_OPTICAL_FLOW_ConsumeByte(dev, byte);
        if (((dev->frames - start_frames) >= min_frames) &&
            (dev->latest.valid == DRV_OPTICAL_FLOW_VALID)) {
            flow_reset_parser(dev);
            return 1U;
        }
    }

    flow_reset_parser(dev);
    return 0U;
}

static uint8_t flow_lc307_expect_status(DRV_OPTICAL_FLOW_Device *dev,
                                        uint8_t command,
                                        uint8_t *capture,
                                        uint32_t timeout_ms)
{
    uint8_t window[3] = {0U, 0U, 0U};
    uint32_t received = 0U;
    uint32_t effective_timeout = (timeout_ms != 0U) ? timeout_ms :
                                 flow_timeout_ms(dev);
    uint32_t start_ms = HAL_GetTick();

    if (capture != NULL) {
        capture[0] = 0U;
        capture[1] = 0U;
        capture[2] = 0U;
    }

    for (uint32_t index = 0U; index < LC307_STATUS_SCAN_MAX; ++index) {
        uint8_t byte = 0U;
        HAL_StatusTypeDef status =
            HAL_UART_Receive(dev->bus.huart,
                             &byte,
                             1U,
                             LC307_STATUS_BYTE_TIMEOUT_MS);

        dev->config_last_hal_status = (uint32_t)status;
        if (status == HAL_TIMEOUT) {
            if ((HAL_GetTick() - start_ms) >= effective_timeout) {
                return 0U;
            }
            continue;
        }
        if (status != HAL_OK) {
            dev->last_uart_error = dev->bus.huart->ErrorCode;
            return 0U;
        }

        window[0] = window[1];
        window[1] = window[2];
        window[2] = byte;
        if (received < 3U) {
            received++;
        }

        if (capture != NULL) {
            capture[0] = window[0];
            capture[1] = window[1];
            capture[2] = window[2];
        }

        if ((received >= 3U) &&
            (window[0] == command) &&
            (window[1] == 0x00U) &&
            (window[2] == (uint8_t)(window[0] ^ window[1]))) {
            return 1U;
        }
    }

    return 0U;
}

static uint8_t flow_lc307_transmit(DRV_OPTICAL_FLOW_Device *dev,
                                   const uint8_t *data,
                                   uint16_t length)
{
    return (HAL_UART_Transmit(dev->bus.huart, (uint8_t *)data, length,
                              flow_timeout_ms(dev)) == HAL_OK) ? 1U : 0U;
}

static void flow_lc307_configure(DRV_OPTICAL_FLOW_Device *dev)
{
    static const uint8_t ab_command[] = {
        LC307_CMD_AB, 0x96U, 0x26U, 0xBCU, 0x50U, 0x5CU
    };
    const uint8_t aa_command = LC307_CMD_AA;
    const uint8_t dd_command = LC307_CMD_DD;
    const uint32_t table_count =
        (uint32_t)(sizeof(lc307_config_table) / sizeof(lc307_config_table[0]));

    dev->config_attempted = 1U;
    dev->config_status = DRV_OPTICAL_FLOW_CONFIG_ERROR;
    dev->config_bb_expected = table_count;

    flow_delay_ms(dev, LC307_CONFIG_DELAY_MS);
    flow_lc307_drain_rx(dev);

    for (uint32_t attempt = 0U; attempt < LC307_CONFIG_AB_RETRIES; ++attempt) {
        flow_lc307_drain_rx(dev);
        __HAL_UART_CLEAR_FLAG(dev->bus.huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                           UART_CLEAR_PEF | UART_CLEAR_FEF);
        dev->bus.huart->ErrorCode = HAL_UART_ERROR_NONE;
        if (flow_lc307_transmit(dev, &aa_command, 1U) == 0U) {
            dev->config_errors++;
            dev->config_last_error = LC307_ERR_AB_TX;
            return;
        }

        if (flow_lc307_transmit(dev, ab_command, sizeof(ab_command)) == 0U) {
            dev->config_errors++;
            dev->config_last_error = LC307_ERR_AB_TX;
            return;
        }

        if (flow_lc307_expect_status(dev, LC307_CMD_AB,
                                     dev->config_ab_response,
                                     flow_timeout_ms(dev)) != 0U) {
            dev->config_ab_ok = 1U;
            break;
        }

        flow_delay_ms(dev, 10U);
    }

    if (dev->config_ab_ok == 0U) {
        dev->config_errors++;
        dev->config_last_error = LC307_ERR_AB_RX;
        return;
    }

    if (table_count == 0U) {
        dev->config_missing_table = 1U;
        dev->config_status = DRV_OPTICAL_FLOW_CONFIG_MISSING_TABLE;
    }

    for (uint32_t i = 0U; i < table_count; ++i) {
        const uint8_t bb_command[] = {
            LC307_CMD_BB,
            LC307_SENSOR_IIC_ADDR,
            lc307_config_table[i].reg,
            lc307_config_table[i].value,
            (uint8_t)(LC307_SENSOR_IIC_ADDR ^ lc307_config_table[i].reg ^
                      lc307_config_table[i].value)
        };

        uint8_t bb_ok = 0U;
        for (uint32_t attempt = 0U; attempt < LC307_CONFIG_BB_RETRIES; ++attempt) {
            flow_lc307_drain_rx(dev);
            __HAL_UART_CLEAR_FLAG(dev->bus.huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                               UART_CLEAR_PEF | UART_CLEAR_FEF);
            dev->bus.huart->ErrorCode = HAL_UART_ERROR_NONE;

            if (flow_lc307_transmit(dev, bb_command, sizeof(bb_command)) == 0U) {
                dev->config_errors++;
                dev->config_bb_errors++;
                dev->config_last_error = LC307_ERR_BB_TX;
                return;
            }
            dev->config_bb_sent++;

            if (flow_lc307_expect_status(dev, LC307_CMD_BB,
                                         dev->config_bb_response,
                                         LC307_CONFIG_BB_TIMEOUT_MS) != 0U) {
                dev->config_bb_ok++;
                bb_ok = 1U;
                break;
            }

            flow_delay_ms(dev, LC307_CONFIG_BB_GAP_MS);
        }

        if (bb_ok == 0U) {
            dev->config_errors++;
            dev->config_bb_errors++;
            dev->config_last_error = LC307_ERR_BB_RX;
            return;
        }

        if ((lc307_config_table[i].reg == 0x12U) &&
            (lc307_config_table[i].value == 0x80U)) {
            flow_delay_ms(dev, LC307_CONFIG_RESET_MS);
        } else {
            flow_delay_ms(dev, LC307_CONFIG_BB_GAP_MS);
        }
    }

    if (flow_lc307_transmit(dev, &dd_command, 1U) == 0U) {
        dev->config_errors++;
        dev->config_last_error = LC307_ERR_DD_TX;
        return;
    }

    if (table_count != 0U) {
        dev->config_status = DRV_OPTICAL_FLOW_CONFIG_OK;
    }
}

static uint8_t flow_accept_frame(DRV_OPTICAL_FLOW_Device *dev)
{
    uint8_t xor_sum = 0U;

    for (uint32_t i = 2U; i <= 11U; ++i) {
        xor_sum ^= dev->frame[i];
    }

    if (xor_sum != dev->frame[12]) {
        dev->checksum_errors++;
        return 0U;
    }

    dev->latest.flow_x_integral =
        (int16_t)flow_get_u16_le(&dev->frame[2]);
    dev->latest.flow_y_integral =
        (int16_t)flow_get_u16_le(&dev->frame[4]);
    dev->latest.integration_timespan_us = flow_get_u16_le(&dev->frame[6]);
    dev->latest.ground_distance = flow_get_u16_le(&dev->frame[8]);
    dev->latest.valid = dev->frame[10];
    dev->latest.version = dev->frame[11];
    dev->latest.received_ms = HAL_GetTick();
    dev->last_rx_ms = dev->latest.received_ms;
    flow_update_raw_stats(dev, &dev->latest);
    dev->frames++;
    return 1U;
}

uint8_t DRV_OPTICAL_FLOW_ConsumeByte(DRV_OPTICAL_FLOW_Device *dev, uint8_t byte)
{
    if (dev == NULL) {
        return 0U;
    }

    dev->bytes++;

    if (dev->offset == 0U) {
        if (byte != 0xFEU) {
            return 0U;
        }
        dev->frame[dev->offset++] = byte;
        return 0U;
    }

    if (dev->offset == 1U) {
        if (byte != 0x0AU) {
            dev->frame_errors++;
            flow_reset_parser(dev);
            if (byte == 0xFEU) {
                dev->frame[dev->offset++] = byte;
            }
            return 0U;
        }
        dev->frame[dev->offset++] = byte;
        return 0U;
    }

    dev->frame[dev->offset++] = byte;
    if (dev->offset < DRV_OPTICAL_FLOW_FRAME_LEN) {
        return 0U;
    }

    flow_reset_parser(dev);
    if (dev->frame[13] != 0x55U) {
        dev->frame_errors++;
        if (byte == 0xFEU) {
            dev->frame[dev->offset++] = byte;
        }
        return 0U;
    }

    return flow_accept_frame(dev);
}

DRV_OPTICAL_FLOW_Status DRV_OPTICAL_FLOW_Init(DRV_OPTICAL_FLOW_Device *dev,
                                              const DRV_OPTICAL_FLOW_Bus *bus)
{
    UART_HandleTypeDef *huart;

    if ((dev == NULL) || (bus == NULL) || (bus->huart == NULL)) {
        return DRV_OPTICAL_FLOW_INVALID_ARG;
    }

    huart = bus->huart;
    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;
    if (dev->bus.baud_rate == 0U) {
        dev->bus.baud_rate = DRV_OPTICAL_FLOW_BAUD_RATE;
    }

    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_DeInit(huart);
    huart->Init.BaudRate = dev->bus.baud_rate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    if (HAL_UART_Init(huart) != HAL_OK) {
        dev->last_uart_error = huart->ErrorCode;
        dev->uart_errors++;
        return DRV_OPTICAL_FLOW_ERROR;
    }

    if (flow_lc307_probe_stream(dev,
                                LC307_STREAM_PROBE_MS,
                                LC307_STREAM_PROBE_FRAMES) != 0U) {
        dev->config_status = DRV_OPTICAL_FLOW_CONFIG_OK;
    } else {
        flow_lc307_configure(dev);
    }
    dev->initialized = 1U;
    flow_restart_rx(dev);
    return (dev->config_status == DRV_OPTICAL_FLOW_CONFIG_OK) ?
           DRV_OPTICAL_FLOW_OK : DRV_OPTICAL_FLOW_ERROR;
}

void DRV_OPTICAL_FLOW_Service(DRV_OPTICAL_FLOW_Device *dev)
{
    if ((dev == NULL) || (dev->bus.huart == NULL) || (dev->initialized == 0U)) {
        return;
    }

    if (dev->bus.huart->RxState != HAL_UART_STATE_BUSY_RX) {
        flow_restart_rx(dev);
    }
}

void DRV_OPTICAL_FLOW_OnUartRxCplt(DRV_OPTICAL_FLOW_Device *dev)
{
    if ((dev == NULL) || (dev->initialized == 0U)) {
        return;
    }

    (void)DRV_OPTICAL_FLOW_ConsumeByte(dev, dev->rx_byte);
    flow_restart_rx(dev);
}

void DRV_OPTICAL_FLOW_OnUartRxEvent(DRV_OPTICAL_FLOW_Device *dev,
                                    uint16_t size)
{
    uint16_t write_pos;

    if ((dev == NULL) || (dev->initialized == 0U) ||
        (dev->bus.rx_dma_buffer == NULL) || (dev->bus.rx_dma_size == 0U)) {
        return;
    }

    dev->dma_events++;
    dev->dma_last_size = (uint32_t)size;
    if (dev->bus.cache_invalidate != NULL) {
        dev->bus.cache_invalidate(dev->bus.rx_dma_buffer,
                                  dev->bus.rx_dma_size);
    }

    write_pos = flow_dma_write_pos(dev, size);
    if (write_pos >= dev->bus.rx_dma_size) {
        write_pos = 0U;
    }

    while (dev->rx_dma_pos != write_pos) {
        (void)DRV_OPTICAL_FLOW_ConsumeByte(dev,
                                           dev->bus.rx_dma_buffer[dev->rx_dma_pos]);
        dev->rx_dma_pos++;
        if (dev->rx_dma_pos >= dev->bus.rx_dma_size) {
            dev->rx_dma_pos = 0U;
        }
    }
}

void DRV_OPTICAL_FLOW_OnUartError(DRV_OPTICAL_FLOW_Device *dev,
                                  uint32_t error_code)
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
    flow_abort_rx(dev);
    flow_reset_parser(dev);
    flow_restart_rx(dev);
}

void DRV_OPTICAL_FLOW_Invalidate(DRV_OPTICAL_FLOW_Device *dev)
{
    if (dev == NULL) {
        return;
    }

    if (dev->bus.huart != NULL) {
        flow_abort_rx(dev);
    }
    dev->initialized = 0U;
    dev->rx_active = 0U;
    flow_reset_parser(dev);
}

DRV_OPTICAL_FLOW_Status DRV_OPTICAL_FLOW_TransmitRaw(DRV_OPTICAL_FLOW_Device *dev,
                                                     const uint8_t *data,
                                                     uint16_t length,
                                                     uint32_t timeout_ms)
{
    UART_HandleTypeDef *huart;
    HAL_StatusTypeDef status;

    if ((dev == NULL) || (data == NULL) || (length == 0U) ||
        (dev->bus.huart == NULL)) {
        return DRV_OPTICAL_FLOW_INVALID_ARG;
    }

    huart = dev->bus.huart;
    flow_abort_rx(dev);
    flow_reset_parser(dev);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                 UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    status = HAL_UART_Transmit(huart, (uint8_t *)data, length,
                               (timeout_ms != 0U) ? timeout_ms :
                               flow_timeout_ms(dev));
    if (status != HAL_OK) {
        dev->uart_errors++;
        dev->last_uart_error = huart->ErrorCode;
        flow_restart_rx(dev);
        return DRV_OPTICAL_FLOW_ERROR;
    }

    flow_restart_rx(dev);
    return DRV_OPTICAL_FLOW_OK;
}

uint16_t DRV_OPTICAL_FLOW_ReceiveRaw(DRV_OPTICAL_FLOW_Device *dev,
                                     uint8_t *data,
                                     uint16_t max_length,
                                     uint32_t per_byte_timeout_ms)
{
    UART_HandleTypeDef *huart;
    uint16_t count = 0U;

    if ((dev == NULL) || (data == NULL) || (max_length == 0U) ||
        (dev->bus.huart == NULL)) {
        return 0U;
    }

    huart = dev->bus.huart;
    flow_abort_rx(dev);
    flow_reset_parser(dev);

    while (count < max_length) {
        HAL_StatusTypeDef status =
            HAL_UART_Receive(huart, &data[count], 1U,
                             (per_byte_timeout_ms != 0U) ?
                             per_byte_timeout_ms : flow_timeout_ms(dev));
        if (status != HAL_OK) {
            dev->last_uart_error = huart->ErrorCode;
            break;
        }
        count++;
    }

    flow_restart_rx(dev);
    return count;
}

DRV_OPTICAL_FLOW_Status DRV_OPTICAL_FLOW_TransceiveRaw(DRV_OPTICAL_FLOW_Device *dev,
                                                       const uint8_t *tx_data,
                                                       uint16_t tx_length,
                                                       uint8_t *rx_data,
                                                       uint16_t rx_length,
                                                       uint16_t *rx_count,
                                                       uint32_t timeout_ms)
{
    UART_HandleTypeDef *huart;
    HAL_StatusTypeDef status;
    uint16_t count = 0U;
    uint32_t effective_timeout;

    if (rx_count != NULL) {
        *rx_count = 0U;
    }

    if ((dev == NULL) || (tx_data == NULL) || (tx_length == 0U) ||
        (rx_data == NULL) || (rx_length == 0U) || (dev->bus.huart == NULL)) {
        return DRV_OPTICAL_FLOW_INVALID_ARG;
    }

    huart = dev->bus.huart;
    effective_timeout = (timeout_ms != 0U) ? timeout_ms : flow_timeout_ms(dev);
    flow_abort_rx(dev);
    flow_reset_parser(dev);
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                 UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    status = HAL_UART_Transmit(huart, (uint8_t *)tx_data, tx_length,
                               effective_timeout);
    if (status != HAL_OK) {
        dev->uart_errors++;
        dev->last_uart_error = huart->ErrorCode;
        flow_restart_rx(dev);
        return DRV_OPTICAL_FLOW_ERROR;
    }

    while (count < rx_length) {
        status = HAL_UART_Receive(huart, &rx_data[count], 1U,
                                  effective_timeout);
        if (status != HAL_OK) {
            dev->last_uart_error = huart->ErrorCode;
            break;
        }
        count++;
    }

    if (rx_count != NULL) {
        *rx_count = count;
    }
    flow_restart_rx(dev);
    return DRV_OPTICAL_FLOW_OK;
}
