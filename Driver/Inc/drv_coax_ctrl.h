#ifndef DRV_COAX_CTRL_H
#define DRV_COAX_CTRL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_COAX_CTRL_STATE_LEN 18U
#define DRV_COAX_CTRL_REF_LEN   4U
#define DRV_COAX_CTRL_CMD_LEN   4U

#define DRV_COAX_CTRL_SERVO_CENTER_US 1500U
#define DRV_COAX_CTRL_SERVO_MIN_US     500U
#define DRV_COAX_CTRL_SERVO_MAX_US    2500U

typedef struct {
    float x_m;
    float y_m;
    float z_m;
    float yaw_rad;
} DRV_COAX_CTRL_Reference;

typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
} DRV_COAX_CTRL_AttitudeInput;

typedef struct {
    float omega_upper;
    float omega_lower;
    float alpha_rad;
    float beta_rad;
    uint16_t servo_alpha_us;
    uint16_t servo_beta_us;
} DRV_COAX_CTRL_Output;

void DRV_COAX_CTRL_Init(void);

void DRV_COAX_CTRL_Run(const DRV_COAX_CTRL_AttitudeInput *attitude,
                       const DRV_COAX_CTRL_Reference *reference,
                       DRV_COAX_CTRL_Output *output);

uint16_t DRV_COAX_CTRL_TiltRadToServoPulse(float tilt_rad);
uint16_t DRV_COAX_CTRL_OmegaToMotorPulse(float omega_rad_s);

#ifdef __cplusplus
}
#endif

#endif
