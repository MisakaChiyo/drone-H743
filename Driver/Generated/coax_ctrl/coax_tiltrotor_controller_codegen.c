/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: coax_tiltrotor_controller_codegen.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-05-20 17:56:42
 */

/* Include Files */
#include "coax_tiltrotor_controller_codegen.h"
#include "coax_tiltrotor_controller_codegen_data.h"
#include "coax_tiltrotor_controller_codegen_initialize.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static float rt_atan2f_snf(float u0, float u1);

/* Function Definitions */
/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_atan2f_snf(float u0, float u1)
{
  float y;
  int i;
  int i1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0F) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = (float)atan2((float)i, (float)i1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (float)atan2(u0, u1);
  }
  return y;
}

/*
 * x_rb:   18x1 single, [rE; vB; reshape(R_EB,9,1); omegaB]
 *  ref_cmd: 4x1 single, [x_ref; y_ref; z_ref; psi_ref]
 *  cmd:     4x1 single, [Omega_u; Omega_l; alpha; beta]
 *
 * Arguments    : const float x_rb[18]
 *                const float ref_cmd[4]
 *                const DRV_COAX_CTRL_Params *params
 *                float cmd[4]
 * Return Type  : void
 */
void coax_tiltrotor_controller_codegen(const float x_rb[18],
                                       const float ref_cmd[4],
                                       const DRV_COAX_CTRL_Params *params,
                                       float cmd[4])
{
  float fv[3];
  float fv1[3];
  static const signed char b_y[9] = {3, 0, 0, 0, 3, 0, 0, 0, 3};
  float R_EB[9];
  float b_x_rb[9];
  float c_x_rb[9];
  float y[9];
  float FT_des_B[3];
  float acc_cmd_E[3];
  float T_des;
  float alpha_cmd;
  float b_u0;
  float beta_cmd;
  float q;
  float u0;
  float x_rb_tmp;
  int b_i;
  int i;
  int y_tmp;
  boolean_T rEQ0;
  DRV_COAX_CTRL_Params default_params;
  if (!isInitialized_coax_tiltrotor_controller_codegen) {
    coax_tiltrotor_controller_codegen_initialize();
  }
  if (params == NULL) {
    DRV_COAX_CTRL_GetDefaultParams(&default_params);
    params = &default_params;
  }
  fv[0] = params->pos_x_kp;
  fv[1] = params->pos_y_kp;
  fv[2] = params->pos_z_kp;
  fv1[0] = params->vel_x_kd;
  fv1[1] = params->vel_y_kd;
  fv1[2] = params->vel_z_kd;
  for (i = 0; i < 9; i++) {
    x_rb_tmp = x_rb[i + 6];
    b_x_rb[i] = x_rb_tmp;
    R_EB[i] = x_rb_tmp;
    c_x_rb[i] = x_rb_tmp;
  }
  for (i = 0; i < 3; i++) {
    alpha_cmd = R_EB[3 * i];
    x_rb_tmp = R_EB[3 * i + 1];
    q = R_EB[3 * i + 2];
    for (b_i = 0; b_i < 3; b_i++) {
      y_tmp = i + 3 * b_i;
      y[y_tmp] =
          (float)b_y[y_tmp] -
          ((alpha_cmd * c_x_rb[3 * b_i] + x_rb_tmp * c_x_rb[3 * b_i + 1]) +
           q * c_x_rb[3 * b_i + 2]);
    }
  }
  for (i = 0; i < 3; i++) {
    alpha_cmd = b_x_rb[i];
    x_rb_tmp = b_x_rb[i + 3];
    q = b_x_rb[i + 6];
    T_des = 0.0F;
    for (b_i = 0; b_i < 3; b_i++) {
      b_u0 =
          (params->rotation_error_gain * alpha_cmd * y[3 * b_i] +
           params->rotation_error_gain * x_rb_tmp * y[3 * b_i + 1]) +
          params->rotation_error_gain * q * y[3 * b_i + 2];
      R_EB[i + 3 * b_i] = b_u0;
      T_des += b_u0 * x_rb[b_i + 3];
    }
    acc_cmd_E[i] = fv[i] * (ref_cmd[i] - x_rb[i]) - fv1[i] * T_des;
  }
  if (R_EB[2] >= -1.0F) {
    u0 = R_EB[2];
  } else {
    u0 = -1.0F;
  }
  if (acc_cmd_E[0] >= -params->accel_xy_limit_m_s2) {
    alpha_cmd = acc_cmd_E[0];
  } else {
    alpha_cmd = -params->accel_xy_limit_m_s2;
  }
  if (alpha_cmd <= params->accel_xy_limit_m_s2) {
    acc_cmd_E[0] = alpha_cmd;
  } else {
    acc_cmd_E[0] = params->accel_xy_limit_m_s2;
  }
  if (acc_cmd_E[1] >= -params->accel_xy_limit_m_s2) {
    alpha_cmd = acc_cmd_E[1];
  } else {
    alpha_cmd = -params->accel_xy_limit_m_s2;
  }
  if (acc_cmd_E[2] >= -params->accel_z_limit_m_s2) {
    b_u0 = acc_cmd_E[2];
  } else {
    b_u0 = -params->accel_z_limit_m_s2;
  }
  q = params->mass_kg * acc_cmd_E[0];
  if (!(alpha_cmd <= params->accel_xy_limit_m_s2)) {
    alpha_cmd = params->accel_xy_limit_m_s2;
  }
  T_des = params->mass_kg * alpha_cmd;
  if (!(b_u0 <= params->accel_z_limit_m_s2)) {
    b_u0 = params->accel_z_limit_m_s2;
  }
  x_rb_tmp = params->mass_kg * (b_u0 - params->gravity_m_s2);
  for (i = 0; i < 3; i++) {
    alpha_cmd = (R_EB[3 * i] * q + R_EB[3 * i + 1] * T_des) +
                R_EB[3 * i + 2] * x_rb_tmp;
    FT_des_B[i] = alpha_cmd;
    acc_cmd_E[i] = alpha_cmd * alpha_cmd;
  }
  T_des = (float)sqrt((acc_cmd_E[0] + acc_cmd_E[1]) + acc_cmd_E[2]);
  if (T_des < 1.0E-6F) {
    FT_des_B[0] = 0.0F;
    FT_des_B[1] = 0.0F;
    FT_des_B[2] = 1.0F;
  } else {
    FT_des_B[0] = -FT_des_B[0] / T_des;
    FT_des_B[1] = -FT_des_B[1] / T_des;
    FT_des_B[2] = -FT_des_B[2] / T_des;
  }
  b_u0 = (float)sqrt((FT_des_B[0] * FT_des_B[0] + FT_des_B[1] * FT_des_B[1]) +
                     FT_des_B[2] * FT_des_B[2]);
  if (b_u0 >= 1.0E-6F) {
    x_rb_tmp = b_u0;
  } else {
    x_rb_tmp = 1.0E-6F;
  }
  FT_des_B[0] /= x_rb_tmp;
  FT_des_B[1] /= x_rb_tmp;
  FT_des_B[2] /= x_rb_tmp;
  if (-FT_des_B[1] >= -1.0F) {
    b_u0 = -FT_des_B[1];
  } else {
    b_u0 = -1.0F;
  }
  if (T_des >= params->min_total_force_n) {
    x_rb_tmp = T_des;
  } else {
    x_rb_tmp = params->min_total_force_n;
  }
  x_rb_tmp *= params->tilt_lever_arm_m;
  if (!(b_u0 <= 1.0F)) {
    b_u0 = 1.0F;
  }
  b_u0 =
      (float)asin(b_u0) +
      (-params->roll_angle_kp * rt_atan2f_snf(R_EB[5], R_EB[8]) -
       params->roll_rate_kd * x_rb[15]) /
          x_rb_tmp;
  if (!(u0 <= 1.0F)) {
    u0 = 1.0F;
  }
  u0 = rt_atan2f_snf(FT_des_B[0], FT_des_B[2]) +
       (-params->pitch_angle_kp * -(float)asin(u0) -
        params->pitch_rate_kd * x_rb[16]) /
           x_rb_tmp;
  if (!(u0 >= -params->tilt_limit_rad)) {
    u0 = -params->tilt_limit_rad;
  }
  if (u0 <= params->tilt_limit_rad) {
    alpha_cmd = u0;
  } else {
    alpha_cmd = params->tilt_limit_rad;
  }
  if (b_u0 >= -params->tilt_limit_rad) {
    u0 = b_u0;
  } else {
    u0 = -params->tilt_limit_rad;
  }
  if (u0 <= params->tilt_limit_rad) {
    beta_cmd = u0;
  } else {
    beta_cmd = params->tilt_limit_rad;
  }
  if (T_des >= params->min_total_force_n) {
    u0 = T_des;
  } else {
    u0 = params->min_total_force_n;
  }
  x_rb_tmp = ref_cmd[3] - rt_atan2f_snf(R_EB[1], R_EB[0]);
  if (rtIsNaNF(x_rb_tmp + 3.14159274F) || rtIsInfF(x_rb_tmp + 3.14159274F)) {
    b_u0 = rtNaNF;
  } else if (x_rb_tmp + 3.14159274F == 0.0F) {
    b_u0 = 0.0F;
  } else {
    b_u0 = (float)fmod(x_rb_tmp + 3.14159274F, 6.2831854820251465);
    rEQ0 = (b_u0 == 0.0F);
    if (!rEQ0) {
      q = (float)fabs((x_rb_tmp + 3.14159274F) / 6.28318548F);
      rEQ0 = !((float)fabs(q - (float)floor(q + 0.5F)) > 1.1920929E-7F * q);
    }
    if (rEQ0) {
      b_u0 = 0.0F;
    } else if (x_rb_tmp + 3.14159274F < 0.0F) {
      b_u0 += 6.28318548F;
    }
  }
  b_u0 = params->yaw_angle_kp * (b_u0 - 3.14159274F);
  if (!(b_u0 >= -params->yaw_rate_limit_rad_s)) {
    b_u0 = -params->yaw_rate_limit_rad_s;
  }
  if (!(u0 <= params->max_total_force_n)) {
    u0 = params->max_total_force_n;
  }
  T_des = u0 / params->thrust_coeff_n_per_rad2;
  if (!(b_u0 <= params->yaw_rate_limit_rad_s)) {
    b_u0 = params->yaw_rate_limit_rad_s;
  }
  x_rb_tmp = params->yaw_inertia * (b_u0 - params->yaw_rate_kd * x_rb[17]) /
             ((float)cos(alpha_cmd) * (float)cos(beta_cmd)) /
             params->yaw_torque_coeff_n_m_per_rad2;
  u0 = 0.5F * (T_des - x_rb_tmp);
  b_u0 = 0.5F * (T_des + x_rb_tmp);
  if (!(u0 >= 0.0F)) {
    u0 = 0.0F;
  }
  if (!(b_u0 >= 0.0F)) {
    b_u0 = 0.0F;
  }
  if (!(u0 <= params->motor_omega_max_rad_s * params->motor_omega_max_rad_s)) {
    u0 = params->motor_omega_max_rad_s * params->motor_omega_max_rad_s;
  }
  cmd[0] = (float)sqrt(u0);
  if (!(b_u0 <= params->motor_omega_max_rad_s * params->motor_omega_max_rad_s)) {
    b_u0 = params->motor_omega_max_rad_s * params->motor_omega_max_rad_s;
  }
  cmd[1] = (float)sqrt(b_u0);
  cmd[2] = alpha_cmd;
  cmd[3] = beta_cmd;
}

/*
 * File trailer for coax_tiltrotor_controller_codegen.c
 *
 * [EOF]
 */
