#ifndef APP_MAG_H
#define APP_MAG_H

#include <stdint.h>

typedef struct {
    uint8_t initialized;
    int32_t init_status;
    int32_t last_status;
    uint8_t type;
    uint8_t address;
    uint8_t who_am_i;
    uint32_t sample_count;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    int32_t x_mgauss;
    int32_t y_mgauss;
    int32_t z_mgauss;
    uint8_t detected_ist8310;
    uint8_t detected_hmc5883;
    uint8_t detected_qmc5883;
    uint8_t hmc_id_a;
    uint8_t hmc_id_b;
    uint8_t hmc_id_c;
} APP_MAG_Status;

void APP_MAG_Init(void);
void APP_MAG_Step(void);
void APP_MAG_GetStatus(APP_MAG_Status *status);
void APP_MAG_Report(void);
const char *APP_MAG_GetTypeName(uint8_t type);

#endif
