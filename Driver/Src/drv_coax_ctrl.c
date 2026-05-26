#include "drv_coax_ctrl.h"

#include "bsp_pwm.h"
#include "coax_tiltrotor_controller_codegen.h"
#include "coax_tiltrotor_controller_codegen_initialize.h"
#include "drv_airframe_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DRV_COAX_CTRL_TILT_LIMIT_RAD 0.174533f
#define DRV_COAX_CTRL_SERVO_TRAVEL_RAD 3.141592654f
#define DRV_COAX_CTRL_SERVO_ALPHA_SIGN    (1.0f)
#define DRV_COAX_CTRL_SERVO_BETA_SIGN    (-1.0f)
#define DRV_COAX_CTRL_MOTOR_OMEGA_MAX_RAD_S 900.0f

typedef struct {
    const char *name;
    uint16_t offset;
} DRV_COAX_CTRL_ParamEntry;

static uint8_t coax_ctrl_initialized;
static DRV_COAX_CTRL_Params coax_ctrl_params;

#define DRV_COAX_CTRL_PARAM_ENTRY(field) \
    { "coax." #field, (uint16_t)offsetof(DRV_COAX_CTRL_Params, field) }

static const DRV_COAX_CTRL_ParamEntry coax_ctrl_param_table[] = {
    DRV_COAX_CTRL_PARAM_ENTRY(pos_x_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(pos_y_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(pos_z_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_x_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_y_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_z_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(rotation_error_gain),
    DRV_COAX_CTRL_PARAM_ENTRY(accel_xy_limit_m_s2),
    DRV_COAX_CTRL_PARAM_ENTRY(accel_z_limit_m_s2),
    DRV_COAX_CTRL_PARAM_ENTRY(mass_kg),
    DRV_COAX_CTRL_PARAM_ENTRY(gravity_m_s2),
    DRV_COAX_CTRL_PARAM_ENTRY(min_total_force_n),
    DRV_COAX_CTRL_PARAM_ENTRY(max_total_force_n),
    DRV_COAX_CTRL_PARAM_ENTRY(tilt_lever_arm_m),
    DRV_COAX_CTRL_PARAM_ENTRY(roll_angle_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(roll_rate_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(pitch_angle_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(pitch_rate_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(tilt_limit_rad),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_angle_kp),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_rate_kd),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_rate_limit_rad_s),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_inertia),
    DRV_COAX_CTRL_PARAM_ENTRY(thrust_coeff_n_per_rad2),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_torque_coeff_n_m_per_rad2),
    DRV_COAX_CTRL_PARAM_ENTRY(motor_omega_max_rad_s),
};

static const uint32_t coax_ctrl_param_count =
    sizeof(coax_ctrl_param_table) / sizeof(coax_ctrl_param_table[0]);

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

static float *coax_ctrl_param_ptr(DRV_COAX_CTRL_Params *params,
                                  const DRV_COAX_CTRL_ParamEntry *entry)
{
    return (float *)((uint8_t *)params + entry->offset);
}

static const DRV_COAX_CTRL_ParamEntry *coax_ctrl_find_param(const char *name)
{
    if (name == NULL) {
        return NULL;
    }

    for (uint32_t i = 0U; i < coax_ctrl_param_count; ++i) {
        if (strcmp(name, coax_ctrl_param_table[i].name) == 0) {
            return &coax_ctrl_param_table[i];
        }
    }

    return NULL;
}

static uint8_t coax_ctrl_param_value_valid(const DRV_COAX_CTRL_ParamEntry *entry,
                                           float value)
{
    if ((entry == NULL) || !isfinite(value)) {
        return 0U;
    }

    if (fabsf(value) > 2000.0f) {
        return 0U;
    }

    if ((entry->offset == offsetof(DRV_COAX_CTRL_Params, mass_kg)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, gravity_m_s2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, min_total_force_n)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, max_total_force_n)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, tilt_lever_arm_m)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, tilt_limit_rad)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, yaw_inertia)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, thrust_coeff_n_per_rad2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, yaw_torque_coeff_n_m_per_rad2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, motor_omega_max_rad_s))) {
        return (value > 0.0f) ? 1U : 0U;
    }

    if ((entry->offset == offsetof(DRV_COAX_CTRL_Params, accel_xy_limit_m_s2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, accel_z_limit_m_s2)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, yaw_rate_limit_rad_s))) {
        return (value >= 0.0f) ? 1U : 0U;
    }

    return 1U;
}

static uint8_t coax_ctrl_params_valid(const DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return 0U;
    }

    for (uint32_t i = 0U; i < coax_ctrl_param_count; ++i) {
        if (coax_ctrl_param_value_valid(&coax_ctrl_param_table[i],
                                        *coax_ctrl_param_ptr((DRV_COAX_CTRL_Params *)params,
                                                             &coax_ctrl_param_table[i])) == 0U) {
            return 0U;
        }
    }

    return (params->min_total_force_n <= params->max_total_force_n) ? 1U : 0U;
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
        DRV_COAX_CTRL_ResetParams();
        coax_tiltrotor_controller_codegen_initialize();
        coax_ctrl_initialized = 1U;
    }
}

void DRV_COAX_CTRL_GetDefaultParams(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    params->pos_x_kp = 2.2f;
    params->pos_y_kp = 2.2f;
    params->pos_z_kp = 3.8f;
    params->vel_x_kd = 2.5f;
    params->vel_y_kd = 2.5f;
    params->vel_z_kd = 3.1f;
    params->rotation_error_gain = 0.5f;
    params->accel_xy_limit_m_s2 = 3.0f;
    params->accel_z_limit_m_s2 = 2.5f;
    params->mass_kg = DRV_AIRFRAME_MASS_KG;
    params->gravity_m_s2 = DRV_AIRFRAME_GRAVITY_M_S2;
    params->min_total_force_n = DRV_AIRFRAME_WEIGHT_N;
    params->max_total_force_n = DRV_AIRFRAME_MAX_TOTAL_FORCE_N;
    params->tilt_lever_arm_m = 0.18f;
    params->roll_angle_kp = 0.10f;//6.5
    params->roll_rate_kd = 0.0f;//0.7
    params->pitch_angle_kp = 0.10f;//6.5
    params->pitch_rate_kd = 0.0f;//0.7

    params->tilt_limit_rad = DRV_COAX_CTRL_TILT_LIMIT_RAD;
    params->yaw_angle_kp = 0.8f;
    params->yaw_rate_kd = 1.0f;
    params->yaw_rate_limit_rad_s = 1.04719758f;
    params->yaw_inertia = 0.52f;
    params->thrust_coeff_n_per_rad2 = 3.0e-5f;
    params->yaw_torque_coeff_n_m_per_rad2 = 1.5e-6f;
    params->motor_omega_max_rad_s = DRV_COAX_CTRL_MOTOR_OMEGA_MAX_RAD_S;
}

void DRV_COAX_CTRL_ResetParams(void)
{
    DRV_COAX_CTRL_GetDefaultParams(&coax_ctrl_params);
}

void DRV_COAX_CTRL_GetParams(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    DRV_COAX_CTRL_Init();
    *params = coax_ctrl_params;
}

void DRV_COAX_CTRL_SetParams(const DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    DRV_COAX_CTRL_Init();
    if (coax_ctrl_params_valid(params) != 0U) {
        coax_ctrl_params = *params;
    } else {
        DRV_COAX_CTRL_ResetParams();
    }
}

uint32_t DRV_COAX_CTRL_ParamCount(void)
{
    return coax_ctrl_param_count;
}

const char *DRV_COAX_CTRL_ParamName(uint32_t index)
{
    return (index < coax_ctrl_param_count) ? coax_ctrl_param_table[index].name : NULL;
}

uint8_t DRV_COAX_CTRL_GetParam(const char *name, float *value)
{
    const DRV_COAX_CTRL_ParamEntry *entry = coax_ctrl_find_param(name);

    if ((entry == NULL) || (value == NULL)) {
        return 0U;
    }

    DRV_COAX_CTRL_Init();
    *value = *coax_ctrl_param_ptr(&coax_ctrl_params, entry);
    return 1U;
}

uint8_t DRV_COAX_CTRL_SetParam(const char *name, float value)
{
    const DRV_COAX_CTRL_ParamEntry *entry = coax_ctrl_find_param(name);
    DRV_COAX_CTRL_Params candidate;

    if (coax_ctrl_param_value_valid(entry, value) == 0U) {
        return 0U;
    }

    DRV_COAX_CTRL_Init();
    candidate = coax_ctrl_params;
    *coax_ctrl_param_ptr(&candidate, entry) = value;
    if (coax_ctrl_params_valid(&candidate) == 0U) {
        return 0U;
    }

    coax_ctrl_params = candidate;
    return 1U;
}

static uint16_t coax_ctrl_tilt_rad_to_servo_pulse(float tilt_rad,
                                                  uint16_t center_us)
{
    const float servo_span_us = (float)(DRV_COAX_CTRL_SERVO_MAX_US -
                                        DRV_COAX_CTRL_SERVO_MIN_US);
    const float servo_us_per_rad = servo_span_us / DRV_COAX_CTRL_SERVO_TRAVEL_RAD;
    DRV_COAX_CTRL_Init();
    const float tilt = coax_ctrl_clamp_f32(tilt_rad,
                                          -coax_ctrl_params.tilt_limit_rad,
                                           coax_ctrl_params.tilt_limit_rad);
    const float pulse_f = (float)center_us +
                          tilt * servo_us_per_rad;
    const int32_t pulse_i = (int32_t)(pulse_f + ((pulse_f >= 0.0f) ? 0.5f : -0.5f));

    return coax_ctrl_clamp_u16(pulse_i,
                               DRV_COAX_CTRL_SERVO_MIN_US,
                               DRV_COAX_CTRL_SERVO_MAX_US);
}

uint16_t DRV_COAX_CTRL_AlphaTiltRadToServoPulse(float tilt_rad)
{
    return coax_ctrl_tilt_rad_to_servo_pulse(tilt_rad,
                                             DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US);
}

uint16_t DRV_COAX_CTRL_BetaTiltRadToServoPulse(float tilt_rad)
{
    return coax_ctrl_tilt_rad_to_servo_pulse(tilt_rad,
                                             DRV_COAX_CTRL_SERVO_BETA_CENTER_US);
}

uint16_t DRV_COAX_CTRL_OmegaToMotorPulse(float omega_rad_s)
{
    DRV_COAX_CTRL_Init();
    float omega = coax_ctrl_clamp_f32(omega_rad_s,
                                     0.0f,
                                     coax_ctrl_params.motor_omega_max_rad_s);
    float ratio = omega / coax_ctrl_params.motor_omega_max_rad_s;
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

    coax_tiltrotor_controller_codegen(x_rb, ref_cmd, &coax_ctrl_params, cmd);

    output->omega_upper = cmd[0];
    output->omega_lower = cmd[1];
    output->alpha_rad = cmd[2];
    output->beta_rad = cmd[3];
    output->servo_alpha_us = DRV_COAX_CTRL_AlphaTiltRadToServoPulse(
        cmd[2] * DRV_COAX_CTRL_SERVO_ALPHA_SIGN);
    output->servo_beta_us = DRV_COAX_CTRL_BetaTiltRadToServoPulse(
        cmd[3] * DRV_COAX_CTRL_SERVO_BETA_SIGN);
}
