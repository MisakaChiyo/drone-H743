#ifndef BSP_MAG_H
#define BSP_MAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

typedef enum {
    BSP_MAG_OK = 0,
    BSP_MAG_ERROR,
    BSP_MAG_TIMEOUT,
    BSP_MAG_BAD_ID,
    BSP_MAG_INVALID_ARG,
    BSP_MAG_NOT_READY
} BSP_MAG_StatusCode;

typedef enum {
    BSP_MAG_TYPE_NONE = 0,
    BSP_MAG_TYPE_IST8310,
    BSP_MAG_TYPE_HMC5883,
    BSP_MAG_TYPE_QMC5883L
} BSP_MAG_Type;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} BSP_MAG_RawData;

typedef struct {
    int32_t x_mgauss;
    int32_t y_mgauss;
    int32_t z_mgauss;
} BSP_MAG_ScaledData;

typedef struct {
    BSP_MAG_Type type;
    uint8_t address;
    uint8_t who_am_i;
    uint8_t initialized;
    BSP_MAG_StatusCode last_status;
    uint32_t sample_count;
    BSP_MAG_RawData raw;
    BSP_MAG_ScaledData scaled;
    uint8_t detected_ist8310;
    uint8_t detected_hmc5883;
    uint8_t detected_qmc5883;
    uint8_t hmc_id[3];
} BSP_MAG_Status;

BSP_MAG_StatusCode BSP_MAG_Init(void);
BSP_MAG_StatusCode BSP_MAG_Read(BSP_MAG_RawData *raw, BSP_MAG_ScaledData *scaled);
BSP_MAG_StatusCode BSP_MAG_Probe(BSP_MAG_Status *status);
void BSP_MAG_GetStatus(BSP_MAG_Status *status);
void BSP_MAG_Invalidate(void);
const char *BSP_MAG_TypeName(BSP_MAG_Type type);

#ifdef __cplusplus
}
#endif

#endif
