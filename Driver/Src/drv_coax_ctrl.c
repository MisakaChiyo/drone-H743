#include "drv_coax_ctrl.h"

#include "bsp_pwm.h"
#include "coax_tiltrotor_controller_codegen.h"
#include "coax_tiltrotor_controller_codegen_initialize.h"

#include <math.h>
#include <string.h>

#define DRV_COAX_CTRL_TILT_LIMIT_RAD 0.261799395f
#define DRV_COAX_CTRL_SERVO_TRAVEL_RAD 3.141592654f
#define DRV_COAX_CTRL_SERVO_ALPHA_SIGN    (1.0f)
#define DRV_COAX_CTRL_SERVO_BETA_SIGN    (-1.0f)
#define DRV_COAX_CTRL_MOTOR_OMEGA_MAX_RAD_S 900.0f

static uint8_t coax_ctrl_initialized;

static float coax_ctrl_clamp_f32(float value, float lo, float hi)
{
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
}

static uint16_t coax_ctrl_clamp_u16(int32_t value, uint16_t lo, uint16_t hi)
{
    if (value < (int32_t)lo) { return lo; }
    if (value > (int32_t)hi) { return hi; }
    return (uint16_t)value;
}

static void coax_ctrl_fill_rotation_zyx(float roll_rad,
                                        float pitch_rad,
                                        float yaw_rad,
                                        float state[DRV_COAX_CTRL_STATE_LEN])
{
    const float cr = cosf(roll_rad);
    const float sr = sinf(roll_rad);
    const float cp = cosf(pitch_rad);
    const float sp = sinf(pitch_rad);
    const float cy = cosf(yaw_rad);
    const float sy = sinf(yaw_rad);

    state[6]  = cy * cp;
    state[7]  = sy * cp;
    state[8]  = -sp;
    state[9]  = cy * sp * sr - sy * cr;
    state[10] = sy * sp * sr + cy * cr;
    state[11] = cp * sr;
    state[12] = cy * sp * cr + sy * sr;
    state[13] = sy * sp * cr - cy * sr;
    state[14] = cp * cr;
}

void DRV_COAX_CTRL_Init(void)
{
    if (coax_ctrl_initialized == 0U) {
        coax_tiltrotor_controller_codegen_initialize();
        coax_ctrl_initialized = 1U;
    }
}

uint16_t DRV_COAX_CTRL_TiltRadToServoPulse(float tilt_rad)
{
    const float servo_span_us = (float)(DRV_COAX_CTRL_SERVO_MAX_US -
                                        DRV_COAX_CTRL_SERVO_MIN_US);
    const float servo_us_per_rad = servo_span_us / DRV_COAX_CTRL_SERVO_TRAVEL_RAD;
    const float tilt = coax_ctrl_clamp_f32(tilt_rad,
                                          -DRV_COAX_CTRL_TILT_LIMIT_RAD,
                                           DRV_COAX_CTRL_TILT_LIMIT_RAD);
    const float pulse_f = (float)DRV_COAX_CTRL_SERVO_CENTER_US +
                          tilt * servo_us_per_rad;
    const int32_t pulse_i = (int32_t)(pulse_f + ((pulse_f >= 0.0f) ? 0.5f : -0.5f));

    return coax_ctrl_clamp_u16(pulse_i,
                               DRV_COAX_CTRL_SERVO_MIN_US,
                               DRV_COAX_CTRL_SERVO_MAX_US);
}

uint16_t DRV_COAX_CTRL_OmegaToMotorPulse(float omega_rad_s)
{
    float omega = coax_ctrl_clamp_f32(omega_rad_s,
                                     0.0f,
                                     DRV_COAX_CTRL_MOTOR_OMEGA_MAX_RAD_S);
    float ratio = omega / DRV_COAX_CTRL_MOTOR_OMEGA_MAX_RAD_S;
    float pulse_f = (float)BSP_PWM_ESC_MIN_US +
                    ratio * (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);
    int32_t pulse_i = (int32_t)(pulse_f + 0.5f);

    return coax_ctrl_clamp_u16(pulse_i, BSP_PWM_ESC_MIN_US, BSP_PWM_ESC_MAX_US);
}

void DRV_COAX_CTRL_Run(const DRV_COAX_CTRL_AttitudeInput *attitude,
                       const DRV_COAX_CTRL_Reference *reference,
                       DRV_COAX_CTRL_Output *output)
{
    float x_rb[DRV_COAX_CTRL_STATE_LEN];
    float ref_cmd[DRV_COAX_CTRL_REF_LEN];
    float cmd[DRV_COAX_CTRL_CMD_LEN];

    if ((attitude == NULL) || (reference == NULL) || (output == NULL)) {
        return;
    }

    DRV_COAX_CTRL_Init();

    memset(x_rb, 0, sizeof(x_rb));
    coax_ctrl_fill_rotation_zyx(attitude->roll_rad,
                                attitude->pitch_rad,
                                attitude->yaw_rad,
                                x_rb);
    x_rb[15] = attitude->gyro_x_rad_s;
    x_rb[16] = attitude->gyro_y_rad_s;
    x_rb[17] = attitude->gyro_z_rad_s;

    ref_cmd[0] = reference->x_m;
    ref_cmd[1] = reference->y_m;
    ref_cmd[2] = reference->z_m;
    ref_cmd[3] = reference->yaw_rad;

    coax_tiltrotor_controller_codegen(x_rb, ref_cmd, cmd);

    output->omega_upper = cmd[0];
    output->omega_lower = cmd[1];
    output->alpha_rad = cmd[2];
    output->beta_rad = cmd[3];
    output->servo_alpha_us = DRV_COAX_CTRL_TiltRadToServoPulse(
        cmd[2] * DRV_COAX_CTRL_SERVO_ALPHA_SIGN);
    output->servo_beta_us = DRV_COAX_CTRL_TiltRadToServoPulse(
        cmd[3] * DRV_COAX_CTRL_SERVO_BETA_SIGN);
}
