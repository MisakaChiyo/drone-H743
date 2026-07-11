#ifndef APP_RANGEFINDER_H
#define APP_RANGEFINDER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t initialized;
    uint8_t valid;
    uint16_t distance_cm;
    uint16_t strength;
    float raw_distance_m;
    float height_m;
    float vertical_velocity_m_s;
    float temperature_c;
    uint32_t sample_ms;
    uint32_t age_ms;
    uint32_t bytes;
    uint32_t frames;
    uint32_t checksum_errors;
    uint32_t frame_errors;
    uint32_t rx_restarts;
    uint32_t dma_events;
    uint32_t uart_errors;
    uint32_t last_uart_error;
} APP_RangefinderStatus;

void APP_Rangefinder_Init(void);
void APP_Rangefinder_Step(void);
uint8_t APP_Rangefinder_GetHeightSample(float *height_m,
                                        float *vertical_velocity_m_s,
                                        uint32_t *sample_ms);
void APP_Rangefinder_GetStatus(APP_RangefinderStatus *status);
void APP_Rangefinder_Report(void);

#ifdef __cplusplus
}
#endif

#endif
