#ifndef APP_BARO_H
#define APP_BARO_H

#include <stdint.h>

typedef struct {
    uint8_t report_done;
    int32_t init_status;
    int32_t split_status;
    int32_t txrx_status;
    uint8_t product_id;
    uint8_t split_id;
    uint8_t txrx_id;
    uint8_t bmp280_id;
    uint8_t cs_level;
    uint8_t miso_level;
} APP_Baro_Status;

void APP_Baro_ReportStartup(void);
void APP_Baro_GetStatus(APP_Baro_Status *status);

#endif
