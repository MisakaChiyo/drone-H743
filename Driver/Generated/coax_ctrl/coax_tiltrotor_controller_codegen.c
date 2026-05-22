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
 *                float cmd[4]
 * Return Type  : void
 */
void coax_tiltrotor_controller_codegen(const float x_rb[18],
                                       const float ref_cmd[4], float cmd[4])
{
  static const float fv[3] = {2.2F, 2.2F, 3.8F};
  static const float fv1[3] = {2.5F, 2.5F, 3.1F};
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
  if (!isInitialized_coax_tiltrotor_controller_codegen) {
    coax_tiltrotor_controller_codegen_initialize();
  }
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
          (0.5F * alpha_cmd * y[3 * b_i] + 0.5F * x_rb_tmp * y[3 * b_i + 1]) +
          0.5F * q * y[3 * b_i + 2];
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
  if (acc_cmd_E[0] >= -3.0F) {
    alpha_cmd = acc_cmd_E[0];
  } else {
    alpha_cmd = -3.0F;
  }
  if (alpha_cmd <= 3.0F) {
    acc_cmd_E[0] = alpha_cmd;
  } else {
    acc_cmd_E[0] = 3.0F;
  }
  if (acc_cmd_E[1] >= -3.0F) {
    alpha_cmd = acc_cmd_E[1];
  } else {
    alpha_cmd = -3.0F;
  }
  if (acc_cmd_E[2] >= -2.5F) {
    b_u0 = acc_cmd_E[2];
  } else {
    b_u0 = -2.5F;
  }
  q = 2.2F * acc_cmd_E[0];
  if (!(alpha_cmd <= 3.0F)) {
    alpha_cmd = 3.0F;
  }
  T_des = 2.2F * alpha_cmd;
  if (!(b_u0 <= 2.5F)) {
    b_u0 = 2.5F;
  }
  x_rb_tmp = 2.2F * (b_u0 - 9.81F);
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
  if (T_des >= 7.5537014F) {
    x_rb_tmp = T_des;
  } else {
    x_rb_tmp = 7.5537014F;
  }
  x_rb_tmp *= 0.18F;
  if (!(b_u0 <= 1.0F)) {
    b_u0 = 1.0F;
  }
  b_u0 =
      (float)asin(b_u0) +
      (-6.5F * rt_atan2f_snf(R_EB[5], R_EB[8]) - 0.66F * x_rb[15]) / x_rb_tmp;
  if (!(u0 <= 1.0F)) {
    u0 = 1.0F;
  }
  u0 = rt_atan2f_snf(FT_des_B[0], FT_des_B[2]) +
       (-6.5F * -(float)asin(u0) - 0.7F * x_rb[16]) / x_rb_tmp;
  if (!(u0 >= -0.261799395F)) {
    u0 = -0.261799395F;
  }
  if (u0 <= 0.261799395F) {
    alpha_cmd = u0;
  } else {
    alpha_cmd = 0.261799395F;
  }
  if (b_u0 >= -0.261799395F) {
    u0 = b_u0;
  } else {
    u0 = -0.261799395F;
  }
  if (u0 <= 0.261799395F) {
    beta_cmd = u0;
  } else {
    beta_cmd = 0.261799395F;
  }
  if (T_des >= 7.5537F) {
    u0 = T_des;
  } else {
    u0 = 7.5537F;
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
  b_u0 = 2.8F * (b_u0 - 3.14159274F);
  if (!(b_u0 >= -1.04719758F)) {
    b_u0 = -1.04719758F;
  }
  if (!(u0 <= 38.8476F)) {
    u0 = 38.8476F;
  }
  T_des = u0 / 3.0E-5F;
  if (!(b_u0 <= 1.04719758F)) {
    b_u0 = 1.04719758F;
  }
  x_rb_tmp = 0.52F * (b_u0 - x_rb[17]) /
             ((float)cos(alpha_cmd) * (float)cos(beta_cmd)) / 1.5E-6F;
  u0 = 0.5F * (T_des - x_rb_tmp);
  b_u0 = 0.5F * (T_des + x_rb_tmp);
  if (!(u0 >= 0.0F)) {
    u0 = 0.0F;
  }
  if (!(b_u0 >= 0.0F)) {
    b_u0 = 0.0F;
  }
  if (!(u0 <= 810000.0F)) {
    u0 = 810000.0F;
  }
  cmd[0] = (float)sqrt(u0);
  if (!(b_u0 <= 810000.0F)) {
    b_u0 = 810000.0F;
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
