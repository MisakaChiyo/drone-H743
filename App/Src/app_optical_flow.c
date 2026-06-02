#include "app_optical_flow.h"

#include "app_control.h"
#include "bsp_optical_flow.h"
#include "drv_optical_flow.h"

#include <string.h>

#define APP_FLOW_TIMEOUT_MS          100U
#define APP_FLOW_MIN_DT_US          1000U
#define APP_FLOW_MAX_DT_US          100000U
#define APP_FLOW_FIXED_HEIGHT_M      0.06f
#define APP_FLOW_HEIGHT_LPF_ALPHA   0.00f
#define APP_FLOW_MOUNT_COS_45       0.70710678118f
#define APP_FLOW_MOUNT_SIN_45       0.70710678118f

typedef struct {
    uint8_t initialized;
    int32_t init_status;
    APP_OPTICAL_FLOW_VelSource velocity_source;
    float height_m;
    float height_raw_m;
    uint8_t height_valid;
    float vx_m_s;
    float vy_m_s;
    uint8_t velocity_valid;
    BSP_OPTICAL_FLOW_Status bsp_status;
} APP_OpticalFlow_Context;

static APP_OpticalFlow_Context flow_ctx;

void APP_OpticalFlow_Init(void)
{
    BSP_OPTICAL_FLOW_StatusCode status;

    memset(&flow_ctx, 0, sizeof(flow_ctx));
    flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_IMU;
    flow_ctx.height_m = APP_FLOW_FIXED_HEIGHT_M;
    flow_ctx.height_raw_m = APP_FLOW_FIXED_HEIGHT_M;
    flow_ctx.height_valid = 1U;
    status = BSP_OPTICAL_FLOW_Init();
    flow_ctx.init_status = (int32_t)status;
    flow_ctx.initialized = (status == DRV_OPTICAL_FLOW_OK) ? 1U : 0U;
}

void APP_OpticalFlow_UpdateHeightFromPressure(float pressure_pa, uint8_t fresh)
{
    (void)pressure_pa;
    (void)fresh;
    flow_ctx.height_m = APP_FLOW_FIXED_HEIGHT_M;
    flow_ctx.height_raw_m = APP_FLOW_FIXED_HEIGHT_M;
    flow_ctx.height_valid = 1U;
}

void APP_OpticalFlow_Step(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t age_ms = 0U;
    const BSP_OPTICAL_FLOW_Frame *frame;
    float dt_s;
    float height_m;
    float sensor_vx_m_s;
    float sensor_vy_m_s;
    float body_vx_m_s;
    float body_vy_m_s;

    BSP_OPTICAL_FLOW_Service();
    BSP_OPTICAL_FLOW_GetStatus(&flow_ctx.bsp_status);
    frame = &flow_ctx.bsp_status.latest;

    if (flow_ctx.bsp_status.last_rx_ms != 0U) {
        age_ms = now - flow_ctx.bsp_status.last_rx_ms;
    } else {
        age_ms = 0xFFFFFFFFUL;
    }

    flow_ctx.velocity_valid = 0U;
    if ((frame->valid != DRV_OPTICAL_FLOW_VALID) ||
        (age_ms > APP_FLOW_TIMEOUT_MS) ||
        (frame->integration_timespan_us < APP_FLOW_MIN_DT_US) ||
        (frame->integration_timespan_us > APP_FLOW_MAX_DT_US) ||
        (flow_ctx.height_valid == 0U)) {
        return;
    }

    dt_s = (float)frame->integration_timespan_us * 0.000001f;
    height_m = flow_ctx.height_m;
    sensor_vx_m_s = ((float)frame->flow_x_integral / 10000.0f) *
                    height_m / dt_s;
    sensor_vy_m_s = ((float)frame->flow_y_integral / 10000.0f) *
                    height_m / dt_s;
    body_vx_m_s = APP_FLOW_MOUNT_COS_45 * sensor_vx_m_s -
                  APP_FLOW_MOUNT_SIN_45 * sensor_vy_m_s;
    body_vy_m_s = APP_FLOW_MOUNT_SIN_45 * sensor_vx_m_s +
                  APP_FLOW_MOUNT_COS_45 * sensor_vy_m_s;
    flow_ctx.vx_m_s = -body_vx_m_s;
    flow_ctx.vy_m_s = -body_vy_m_s;
    flow_ctx.velocity_valid = 1U;
}

uint8_t APP_OpticalFlow_GetVelocity(float *vx_m_s, float *vy_m_s)
{
    uint32_t sample_ms;

    return APP_OpticalFlow_GetVelocitySample(vx_m_s, vy_m_s, &sample_ms);
}

uint8_t APP_OpticalFlow_GetVelocitySample(float *vx_m_s,
                                          float *vy_m_s,
                                          uint32_t *sample_ms)
{
    if (flow_ctx.velocity_valid == 0U) {
        flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_IMU;
        return 0U;
    }

    if (vx_m_s != NULL) {
        *vx_m_s = flow_ctx.vx_m_s;
    }
    if (vy_m_s != NULL) {
        *vy_m_s = flow_ctx.vy_m_s;
    }
    if (sample_ms != NULL) {
        *sample_ms = flow_ctx.bsp_status.latest.received_ms;
    }
    flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_FLOW;
    return 1U;
}

void APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VelSource source)
{
    flow_ctx.velocity_source = source;
}

const char *APP_OpticalFlow_VelSourceName(APP_OPTICAL_FLOW_VelSource source)
{
    return (source == APP_OPTICAL_FLOW_VEL_SOURCE_FLOW) ? "flow" : "imu";
}

static const char *flow_config_status_name(uint32_t status)
{
    switch ((DRV_OPTICAL_FLOW_ConfigStatus)status) {
    case DRV_OPTICAL_FLOW_CONFIG_NOT_RUN:
        return "not_run";
    case DRV_OPTICAL_FLOW_CONFIG_OK:
        return "ok";
    case DRV_OPTICAL_FLOW_CONFIG_ERROR:
        return "error";
    case DRV_OPTICAL_FLOW_CONFIG_MISSING_TABLE:
        return "cfg_missing";
    default:
        return "unknown";
    }
}

void APP_OpticalFlow_GetStatus(APP_OPTICAL_FLOW_Status *status)
{
    uint32_t now = HAL_GetTick();
    const BSP_OPTICAL_FLOW_Frame *frame;

    if (status == NULL) {
        return;
    }

    BSP_OPTICAL_FLOW_GetStatus(&flow_ctx.bsp_status);
    frame = &flow_ctx.bsp_status.latest;

    memset(status, 0, sizeof(*status));
    status->initialized = flow_ctx.initialized;
    status->init_status = flow_ctx.init_status;
    status->valid = frame->valid;
    status->version = frame->version;
    status->velocity_valid = flow_ctx.velocity_valid;
    status->velocity_source = flow_ctx.velocity_source;
    status->bytes = flow_ctx.bsp_status.bytes;
    status->frames = flow_ctx.bsp_status.frames;
    status->checksum_errors = flow_ctx.bsp_status.checksum_errors;
    status->frame_errors = flow_ctx.bsp_status.frame_errors;
    status->rx_restarts = flow_ctx.bsp_status.rx_restarts;
    status->dma_events = flow_ctx.bsp_status.dma_events;
    status->dma_last_size = flow_ctx.bsp_status.dma_last_size;
    status->uart_errors = flow_ctx.bsp_status.uart_errors;
    status->last_uart_error = flow_ctx.bsp_status.last_uart_error;
    status->config_status = (uint32_t)flow_ctx.bsp_status.config_status;
    status->config_attempted = flow_ctx.bsp_status.config_attempted;
    status->config_ab_ok = flow_ctx.bsp_status.config_ab_ok;
    status->config_missing_table = flow_ctx.bsp_status.config_missing_table;
    status->config_ab_response[0] = flow_ctx.bsp_status.config_ab_response[0];
    status->config_ab_response[1] = flow_ctx.bsp_status.config_ab_response[1];
    status->config_ab_response[2] = flow_ctx.bsp_status.config_ab_response[2];
    status->config_errors = flow_ctx.bsp_status.config_errors;
    status->config_last_error = flow_ctx.bsp_status.config_last_error;
    status->config_last_hal_status = flow_ctx.bsp_status.config_last_hal_status;
    status->config_bb_expected = flow_ctx.bsp_status.config_bb_expected;
    status->config_bb_sent = flow_ctx.bsp_status.config_bb_sent;
    status->config_bb_ok = flow_ctx.bsp_status.config_bb_ok;
    status->config_bb_errors = flow_ctx.bsp_status.config_bb_errors;
    status->last_rx_ms = flow_ctx.bsp_status.last_rx_ms;
    status->age_ms = (flow_ctx.bsp_status.last_rx_ms != 0U) ?
                     (now - flow_ctx.bsp_status.last_rx_ms) :
                     0xFFFFFFFFUL;
    status->baud_rate = flow_ctx.bsp_status.baud_rate;
    status->flow_x_integral = frame->flow_x_integral;
    status->flow_y_integral = frame->flow_y_integral;
    status->integration_timespan_us = frame->integration_timespan_us;
    status->ground_distance = frame->ground_distance;
    status->raw_count = flow_ctx.bsp_status.raw_stats.count;
    status->flow_x_mean = flow_ctx.bsp_status.raw_stats.flow_x_mean;
    status->flow_y_mean = flow_ctx.bsp_status.raw_stats.flow_y_mean;
    status->integration_timespan_mean_us =
        flow_ctx.bsp_status.raw_stats.integration_timespan_mean_us;
    status->ground_distance_mean =
        flow_ctx.bsp_status.raw_stats.ground_distance_mean;
    status->flow_x_peak_to_peak =
        flow_ctx.bsp_status.raw_stats.flow_x_peak_to_peak;
    status->flow_y_peak_to_peak =
        flow_ctx.bsp_status.raw_stats.flow_y_peak_to_peak;
    status->integration_timespan_peak_to_peak_us =
        flow_ctx.bsp_status.raw_stats.integration_timespan_peak_to_peak_us;
    status->ground_distance_peak_to_peak =
        flow_ctx.bsp_status.raw_stats.ground_distance_peak_to_peak;
    status->height_m = flow_ctx.height_m;
    status->height_raw_m = flow_ctx.height_raw_m;
    status->height_filter_alpha = APP_FLOW_HEIGHT_LPF_ALPHA;
    status->vx_m_s = flow_ctx.vx_m_s;
    status->vy_m_s = flow_ctx.vy_m_s;
}

void APP_OpticalFlow_Report(void)
{
    APP_OPTICAL_FLOW_Status status;
    int32_t height_mm;
    int32_t height_raw_mm;
    int32_t vx_mm_s;
    int32_t vy_mm_s;

    APP_OpticalFlow_GetStatus(&status);
    height_mm = (int32_t)(status.height_m * 1000.0f);
    height_raw_mm = (int32_t)(status.height_raw_m * 1000.0f);
    vx_mm_s = (int32_t)(status.vx_m_s * 1000.0f);
    vy_mm_s = (int32_t)(status.vy_m_s * 1000.0f);
    APP_Control_QueueText("FLOW ok=%u init=%ld baud=%lu bytes=%lu frames=%lu valid=0x%02X ver=%u age_ms=%lu source=%s vel_valid=%u\r\n",
                          (unsigned int)status.initialized,
                          (long)status.init_status,
                          (unsigned long)status.baud_rate,
                          (unsigned long)status.bytes,
                          (unsigned long)status.frames,
                          (unsigned int)status.valid,
                          (unsigned int)status.version,
                          (unsigned long)status.age_ms,
                          APP_OpticalFlow_VelSourceName(status.velocity_source),
                          (unsigned int)status.velocity_valid);
    APP_Control_QueueText("FLOW cfg=%s attempted=%u ab_ok=%u missing_tbl=%u ab_rx=%02X,%02X,%02X hal=%lu bb=%lu/%lu bb_ok=%lu cfg_err=%lu cfg_last=%lu\r\n",
                          flow_config_status_name(status.config_status),
                          (unsigned int)status.config_attempted,
                          (unsigned int)status.config_ab_ok,
                          (unsigned int)status.config_missing_table,
                          (unsigned int)status.config_ab_response[0],
                          (unsigned int)status.config_ab_response[1],
                          (unsigned int)status.config_ab_response[2],
                          (unsigned long)status.config_last_hal_status,
                          (unsigned long)status.config_bb_sent,
                          (unsigned long)status.config_bb_expected,
                          (unsigned long)status.config_bb_ok,
                          (unsigned long)status.config_errors,
                          (unsigned long)status.config_last_error);
    APP_Control_QueueText("FLOW data fx=%d fy=%d dt_us=%u ground=%u height_raw_mm=%ld height_mm=%ld vx_mm_s=%ld vy_mm_s=%ld cksum=%lu frame_err=%lu rst=%lu dma_evt=%lu dma_size=%lu uerr=%lu last_err=0x%lX\r\n",
                          (int)status.flow_x_integral,
                          (int)status.flow_y_integral,
                          (unsigned int)status.integration_timespan_us,
                          (unsigned int)status.ground_distance,
                          (long)height_raw_mm,
                          (long)height_mm,
                          (long)vx_mm_s,
                          (long)vy_mm_s,
                          (unsigned long)status.checksum_errors,
                          (unsigned long)status.frame_errors,
                          (unsigned long)status.rx_restarts,
                          (unsigned long)status.dma_events,
                          (unsigned long)status.dma_last_size,
                          (unsigned long)status.uart_errors,
                          (unsigned long)status.last_uart_error);
    APP_Control_QueueText("FLOW raw n=%u fx_avg=%d fy_avg=%d dt_avg=%u ground_avg=%u fx_pp=%d fy_pp=%d dt_pp=%u ground_pp=%u\r\n",
                          (unsigned int)status.raw_count,
                          (int)status.flow_x_mean,
                          (int)status.flow_y_mean,
                          (unsigned int)status.integration_timespan_mean_us,
                          (unsigned int)status.ground_distance_mean,
                          (int)status.flow_x_peak_to_peak,
                          (int)status.flow_y_peak_to_peak,
                          (unsigned int)status.integration_timespan_peak_to_peak_us,
                          (unsigned int)status.ground_distance_peak_to_peak);
}
