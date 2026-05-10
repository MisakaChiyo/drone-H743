#include "bsp_mag.h"

#include "i2c.h"

#include <string.h>

#define BSP_MAG_I2C_TIMEOUT_MS 20U

#define IST8310_ADDR_7BIT      0x0EU
#define IST8310_WAI_REG        0x00U
#define IST8310_WAI_VALUE      0x10U
#define IST8310_CTRL1_REG      0x0AU
#define IST8310_CTRL2_REG      0x0BU
#define IST8310_CTRL3_REG      0x41U
#define IST8310_DATA_XL_REG    0x03U
#define IST8310_DATA_LEN       6U

#define HMC5883_ADDR_7BIT      0x1EU
#define HMC5883_REG_CFG_A      0x00U
#define HMC5883_REG_CFG_B      0x01U
#define HMC5883_REG_MODE       0x02U
#define HMC5883_REG_DATA_X_MSB 0x03U
#define HMC5883_DATA_LEN       6U
#define HMC5883_IDA_REG        0x0AU
#define HMC5883_IDB_REG        0x0BU
#define HMC5883_IDC_REG        0x0CU

#define QMC5883_ADDR_7BIT      0x0DU
#define QMC5883_REG_XOUT_L     0x00U
#define QMC5883_DATA_LEN       6U
#define QMC5883_REG_CTRL1      0x09U
#define QMC5883_REG_CTRL2      0x0AU
#define QMC5883_REG_CHIP_ID    0x0DU
#define QMC5883_CHIP_ID_VALUE  0xFFU

typedef struct {
    BSP_MAG_Status status;
} BSP_MAG_Context;

static BSP_MAG_Context mag_ctx;

static uint16_t mag_i2c_addr8(uint8_t addr7)
{
    return (uint16_t)((uint16_t)addr7 << 1U);
}

static BSP_MAG_StatusCode mag_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return BSP_MAG_OK;
    case HAL_TIMEOUT:
        return BSP_MAG_TIMEOUT;
    case HAL_ERROR:
    case HAL_BUSY:
    default:
        return BSP_MAG_ERROR;
    }
}

static BSP_MAG_StatusCode mag_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (len == 0U)) {
        return BSP_MAG_INVALID_ARG;
    }

    status = HAL_I2C_Mem_Read(&hi2c1,
                              mag_i2c_addr8(addr7),
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              len,
                              BSP_MAG_I2C_TIMEOUT_MS);
    return mag_from_hal(status);
}

static BSP_MAG_StatusCode mag_i2c_write(uint8_t addr7, uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t data = value;

    status = HAL_I2C_Mem_Write(&hi2c1,
                               mag_i2c_addr8(addr7),
                               reg,
                               I2C_MEMADD_SIZE_8BIT,
                               &data,
                               1U,
                               BSP_MAG_I2C_TIMEOUT_MS);
    return mag_from_hal(status);
}

static int16_t mag_make_i16_le(uint8_t l, uint8_t h)
{
    return (int16_t)((uint16_t)l | ((uint16_t)h << 8U));
}

static int16_t mag_make_i16_be(uint8_t h, uint8_t l)
{
    return (int16_t)(((uint16_t)h << 8U) | (uint16_t)l);
}

static BSP_MAG_StatusCode mag_probe_ist8310(void)
{
    BSP_MAG_StatusCode st;
    uint8_t who = 0U;

    st = mag_i2c_read(IST8310_ADDR_7BIT, IST8310_WAI_REG, &who, 1U);
    if (st != BSP_MAG_OK) {
        return st;
    }
    mag_ctx.status.detected_ist8310 = 1U;
    if (who != IST8310_WAI_VALUE) {
        return BSP_MAG_BAD_ID;
    }

    mag_ctx.status.who_am_i = who;
    mag_ctx.status.address = IST8310_ADDR_7BIT;
    mag_ctx.status.type = BSP_MAG_TYPE_IST8310;
    return BSP_MAG_OK;
}

static BSP_MAG_StatusCode mag_probe_hmc5883(void)
{
    BSP_MAG_StatusCode st;
    uint8_t id[3] = {0U, 0U, 0U};

    st = mag_i2c_read(HMC5883_ADDR_7BIT, HMC5883_IDA_REG, id, 3U);
    if (st != BSP_MAG_OK) {
        return st;
    }

    mag_ctx.status.detected_hmc5883 = 1U;
    mag_ctx.status.hmc_id[0] = id[0];
    mag_ctx.status.hmc_id[1] = id[1];
    mag_ctx.status.hmc_id[2] = id[2];

    if ((id[0] != (uint8_t)'H') || (id[1] != (uint8_t)'4') || (id[2] != (uint8_t)'3')) {
        return BSP_MAG_BAD_ID;
    }

    mag_ctx.status.who_am_i = id[0];
    mag_ctx.status.address = HMC5883_ADDR_7BIT;
    mag_ctx.status.type = BSP_MAG_TYPE_HMC5883;
    return BSP_MAG_OK;
}

static BSP_MAG_StatusCode mag_probe_qmc5883(void)
{
    BSP_MAG_StatusCode st;
    uint8_t id = 0U;

    st = mag_i2c_read(QMC5883_ADDR_7BIT, QMC5883_REG_CHIP_ID, &id, 1U);
    if (st != BSP_MAG_OK) {
        return st;
    }
    mag_ctx.status.detected_qmc5883 = 1U;
    if (id != QMC5883_CHIP_ID_VALUE) {
        return BSP_MAG_BAD_ID;
    }

    mag_ctx.status.who_am_i = id;
    mag_ctx.status.address = QMC5883_ADDR_7BIT;
    mag_ctx.status.type = BSP_MAG_TYPE_QMC5883L;
    return BSP_MAG_OK;
}

static BSP_MAG_StatusCode mag_configure_ist8310(void)
{
    BSP_MAG_StatusCode st;

    st = mag_i2c_write(IST8310_ADDR_7BIT, IST8310_CTRL2_REG, 0x01U);
    if (st != BSP_MAG_OK) {
        return st;
    }
    HAL_Delay(2U);
    st = mag_i2c_write(IST8310_ADDR_7BIT, IST8310_CTRL3_REG, 0x00U);
    if (st != BSP_MAG_OK) {
        return st;
    }

    return mag_i2c_write(IST8310_ADDR_7BIT, IST8310_CTRL1_REG, 0x01U);
}

static BSP_MAG_StatusCode mag_configure_hmc5883(void)
{
    BSP_MAG_StatusCode st;

    st = mag_i2c_write(HMC5883_ADDR_7BIT, HMC5883_REG_CFG_A, 0x18U);
    if (st != BSP_MAG_OK) {
        return st;
    }
    st = mag_i2c_write(HMC5883_ADDR_7BIT, HMC5883_REG_CFG_B, 0x20U);
    if (st != BSP_MAG_OK) {
        return st;
    }

    return mag_i2c_write(HMC5883_ADDR_7BIT, HMC5883_REG_MODE, 0x00U);
}

static BSP_MAG_StatusCode mag_configure_qmc5883(void)
{
    BSP_MAG_StatusCode st;

    st = mag_i2c_write(QMC5883_ADDR_7BIT, QMC5883_REG_CTRL2, 0x01U);
    if (st != BSP_MAG_OK) {
        return st;
    }

    return mag_i2c_write(QMC5883_ADDR_7BIT, QMC5883_REG_CTRL1, 0x1DU);
}

static void mag_scale_data(BSP_MAG_Type type,
                           const BSP_MAG_RawData *raw,
                           BSP_MAG_ScaledData *scaled)
{
    int32_t lsb_per_gauss = 1090;

    if ((raw == NULL) || (scaled == NULL)) {
        return;
    }

    if (type == BSP_MAG_TYPE_QMC5883L) {
        lsb_per_gauss = 12000;
    } else if (type == BSP_MAG_TYPE_IST8310) {
        lsb_per_gauss = 1600;
    }

    scaled->x_mgauss = ((int32_t)raw->x * 1000L) / lsb_per_gauss;
    scaled->y_mgauss = ((int32_t)raw->y * 1000L) / lsb_per_gauss;
    scaled->z_mgauss = ((int32_t)raw->z * 1000L) / lsb_per_gauss;
}

static BSP_MAG_StatusCode mag_read_ist8310(BSP_MAG_RawData *raw)
{
    BSP_MAG_StatusCode st;
    uint8_t data[IST8310_DATA_LEN];

    st = mag_i2c_write(IST8310_ADDR_7BIT, IST8310_CTRL1_REG, 0x01U);
    if (st != BSP_MAG_OK) {
        return st;
    }
    HAL_Delay(10U);
    st = mag_i2c_read(IST8310_ADDR_7BIT, IST8310_DATA_XL_REG, data, IST8310_DATA_LEN);
    if (st != BSP_MAG_OK) {
        return st;
    }

    raw->x = mag_make_i16_le(data[0], data[1]);
    raw->y = mag_make_i16_le(data[2], data[3]);
    raw->z = mag_make_i16_le(data[4], data[5]);
    return BSP_MAG_OK;
}

static BSP_MAG_StatusCode mag_read_hmc5883(BSP_MAG_RawData *raw)
{
    BSP_MAG_StatusCode st;
    uint8_t data[HMC5883_DATA_LEN];

    st = mag_i2c_read(HMC5883_ADDR_7BIT, HMC5883_REG_DATA_X_MSB, data, HMC5883_DATA_LEN);
    if (st != BSP_MAG_OK) {
        return st;
    }

    raw->x = mag_make_i16_be(data[0], data[1]);
    raw->z = mag_make_i16_be(data[2], data[3]);
    raw->y = mag_make_i16_be(data[4], data[5]);
    return BSP_MAG_OK;
}

static BSP_MAG_StatusCode mag_read_qmc5883(BSP_MAG_RawData *raw)
{
    BSP_MAG_StatusCode st;
    uint8_t data[QMC5883_DATA_LEN];

    st = mag_i2c_read(QMC5883_ADDR_7BIT, QMC5883_REG_XOUT_L, data, QMC5883_DATA_LEN);
    if (st != BSP_MAG_OK) {
        return st;
    }

    raw->x = mag_make_i16_le(data[0], data[1]);
    raw->y = mag_make_i16_le(data[2], data[3]);
    raw->z = mag_make_i16_le(data[4], data[5]);
    return BSP_MAG_OK;
}

BSP_MAG_StatusCode BSP_MAG_Init(void)
{
    BSP_MAG_StatusCode st = BSP_MAG_ERROR;

    memset(&mag_ctx, 0, sizeof(mag_ctx));
    mag_ctx.status.type = BSP_MAG_TYPE_NONE;

    st = mag_probe_ist8310();
    if (st == BSP_MAG_OK) {
        st = mag_configure_ist8310();
    }
    if (st != BSP_MAG_OK) {
        st = mag_probe_hmc5883();
        if (st == BSP_MAG_OK) {
            st = mag_configure_hmc5883();
        }
    }
    if (st != BSP_MAG_OK) {
        st = mag_probe_qmc5883();
        if (st == BSP_MAG_OK) {
            st = mag_configure_qmc5883();
        }
    }

    if (st == BSP_MAG_OK) {
        mag_ctx.status.initialized = 1U;
    } else {
        mag_ctx.status.type = BSP_MAG_TYPE_NONE;
        mag_ctx.status.initialized = 0U;
    }
    mag_ctx.status.last_status = st;
    return st;
}

BSP_MAG_StatusCode BSP_MAG_Read(BSP_MAG_RawData *raw, BSP_MAG_ScaledData *scaled)
{
    BSP_MAG_StatusCode st;
    BSP_MAG_RawData local_raw;
    BSP_MAG_ScaledData local_scaled;

    if (mag_ctx.status.initialized == 0U) {
        return BSP_MAG_NOT_READY;
    }

    switch (mag_ctx.status.type) {
    case BSP_MAG_TYPE_IST8310:
        st = mag_read_ist8310(&local_raw);
        break;
    case BSP_MAG_TYPE_HMC5883:
        st = mag_read_hmc5883(&local_raw);
        break;
    case BSP_MAG_TYPE_QMC5883L:
        st = mag_read_qmc5883(&local_raw);
        break;
    default:
        st = BSP_MAG_NOT_READY;
        break;
    }

    if (st != BSP_MAG_OK) {
        mag_ctx.status.last_status = st;
        return st;
    }

    mag_scale_data(mag_ctx.status.type, &local_raw, &local_scaled);
    mag_ctx.status.raw = local_raw;
    mag_ctx.status.scaled = local_scaled;
    mag_ctx.status.sample_count++;
    mag_ctx.status.last_status = BSP_MAG_OK;

    if (raw != NULL) {
        *raw = local_raw;
    }
    if (scaled != NULL) {
        *scaled = local_scaled;
    }

    return BSP_MAG_OK;
}

BSP_MAG_StatusCode BSP_MAG_Probe(BSP_MAG_Status *status)
{
    if (status == NULL) {
        return BSP_MAG_INVALID_ARG;
    }

    *status = mag_ctx.status;
    return mag_ctx.status.last_status;
}

void BSP_MAG_GetStatus(BSP_MAG_Status *status)
{
    if (status == NULL) {
        return;
    }

    *status = mag_ctx.status;
}

void BSP_MAG_Invalidate(void)
{
    memset(&mag_ctx, 0, sizeof(mag_ctx));
    mag_ctx.status.type = BSP_MAG_TYPE_NONE;
}

const char *BSP_MAG_TypeName(BSP_MAG_Type type)
{
    switch (type) {
    case BSP_MAG_TYPE_IST8310:
        return "IST8310";
    case BSP_MAG_TYPE_HMC5883:
        return "HMC5883";
    case BSP_MAG_TYPE_QMC5883L:
        return "QMC5883L";
    default:
        return "NONE";
    }
}
