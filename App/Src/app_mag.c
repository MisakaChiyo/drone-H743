#include "app_mag.h"

#include "bsp_mag.h"

#include <string.h>

typedef struct {
    uint8_t initialized;
    BSP_MAG_StatusCode init_status;
    BSP_MAG_Status bsp_status;
} APP_MAG_Context;

static APP_MAG_Context app_mag_ctx;

void APP_MAG_Init(void)
{
    BSP_MAG_StatusCode st;

    memset(&app_mag_ctx, 0, sizeof(app_mag_ctx));
    st = BSP_MAG_Init();
    app_mag_ctx.init_status = st;
    if (st == BSP_MAG_OK) {
        app_mag_ctx.initialized = 1U;
    }
}

void APP_MAG_Step(void)
{
    BSP_MAG_RawData raw;
    BSP_MAG_ScaledData scaled;

    if (app_mag_ctx.initialized == 0U) {
        return;
    }

    if (BSP_MAG_Read(&raw, &scaled) != BSP_MAG_OK) {
        app_mag_ctx.initialized = 0U;
    }
    BSP_MAG_GetStatus(&app_mag_ctx.bsp_status);
}

void APP_MAG_GetStatus(APP_MAG_Status *status)
{
    const BSP_MAG_Status *bsp;

    if (status == NULL) {
        return;
    }

    BSP_MAG_GetStatus(&app_mag_ctx.bsp_status);
    bsp = &app_mag_ctx.bsp_status;
    memset(status, 0, sizeof(*status));

    status->initialized = app_mag_ctx.initialized;
    status->init_status = (int32_t)app_mag_ctx.init_status;
    status->last_status = (int32_t)bsp->last_status;
    status->type = (uint8_t)bsp->type;
    status->address = bsp->address;
    status->who_am_i = bsp->who_am_i;
    status->sample_count = bsp->sample_count;
    status->raw_x = bsp->raw.x;
    status->raw_y = bsp->raw.y;
    status->raw_z = bsp->raw.z;
    status->x_mgauss = bsp->scaled.x_mgauss;
    status->y_mgauss = bsp->scaled.y_mgauss;
    status->z_mgauss = bsp->scaled.z_mgauss;
    status->detected_ist8310 = bsp->detected_ist8310;
    status->detected_hmc5883 = bsp->detected_hmc5883;
    status->detected_qmc5883 = bsp->detected_qmc5883;
    status->hmc_id_a = bsp->hmc_id[0];
    status->hmc_id_b = bsp->hmc_id[1];
    status->hmc_id_c = bsp->hmc_id[2];
}

const char *APP_MAG_GetTypeName(uint8_t type)
{
    return BSP_MAG_TypeName((BSP_MAG_Type)type);
}
