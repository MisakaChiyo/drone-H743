#ifndef APP_OPTICAL_FLOW_H
#define APP_OPTICAL_FLOW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_OPTICAL_FLOW_VEL_SOURCE_IMU = 0,
    APP_OPTICAL_FLOW_VEL_SOURCE_FLOW = 1
} APP_OPTICAL_FLOW_VelSource;

typedef enum {
    APP_OPTICAL_FLOW_HEALTH_STARTING = 0,
    APP_OPTICAL_FLOW_HEALTH_OK = 1,
    APP_OPTICAL_FLOW_HEALTH_RETRYING = 2,
    APP_OPTICAL_FLOW_HEALTH_FAILED = 3
} APP_OPTICAL_FLOW_Health;

typedef struct {
    uint8_t initialized;
    int32_t init_status;
    APP_OPTICAL_FLOW_Health health;
    uint32_t init_attempts;
    uint32_t recovery_count;
    uint32_t velocity_reject_count;
    uint8_t valid;
    uint8_t version;
    uint8_t velocity_valid;
    APP_OPTICAL_FLOW_VelSource velocity_source;
    uint32_t bytes;
    uint32_t frames;
    uint32_t checksum_errors;
    uint32_t frame_errors;
    uint32_t rx_restarts;
    uint32_t dma_events;
    uint32_t dma_last_size;
    uint32_t uart_errors;
    uint32_t last_uart_error;
    uint32_t config_status;
    uint8_t config_attempted;
    uint8_t config_ab_ok;
    uint8_t config_missing_table;
    uint8_t config_ab_response[3];
    uint8_t config_bb_response[3];
    uint32_t config_errors;
    uint32_t config_last_error;
    uint32_t config_last_hal_status;
    uint32_t config_bb_expected;
    uint32_t config_bb_sent;
    uint32_t config_bb_ok;
    uint32_t config_bb_errors;
    uint32_t last_rx_ms;
    uint32_t age_ms;
    uint32_t baud_rate;
    int16_t flow_x_integral;
    int16_t flow_y_integral;
    uint16_t integration_timespan_us;
    uint16_t ground_distance;
    uint16_t raw_count;
    int16_t flow_x_mean;
    int16_t flow_y_mean;
    uint16_t integration_timespan_mean_us;
    uint16_t ground_distance_mean;
    int16_t flow_x_peak_to_peak;
    int16_t flow_y_peak_to_peak;
    uint16_t integration_timespan_peak_to_peak_us;
    uint16_t ground_distance_peak_to_peak;
    float height_m;
    float height_raw_m;
    float height_filter_alpha;
    float vx_m_s;
    float vy_m_s;
} APP_OPTICAL_FLOW_Status;

void APP_OpticalFlow_Init(void);
void APP_OpticalFlow_Step(void);
void APP_OpticalFlow_ServiceRecovery(void);
void APP_OpticalFlow_UpdateHeightFromPressure(float pressure_pa, uint8_t fresh);
void APP_OpticalFlow_UpdateHeightFromRange(float height_m,
                                           float raw_height_m,
                                           uint8_t valid,
                                           uint32_t sample_ms);
uint8_t APP_OpticalFlow_GetVelocity(float *vx_m_s, float *vy_m_s);
uint8_t APP_OpticalFlow_GetVelocitySample(float *vx_m_s,
                                          float *vy_m_s,
                                          uint32_t *sample_ms);
void APP_OpticalFlow_GetStatus(APP_OPTICAL_FLOW_Status *status);
void APP_OpticalFlow_Report(void);
const char *APP_OpticalFlow_VelSourceName(APP_OPTICAL_FLOW_VelSource source);
void APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VelSource source);

#ifdef __cplusplus
}
#endif

#endif
