/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: coax_tiltrotor_controller_codegen.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2026-05-20 17:56:42
 */

#ifndef COAX_TILTROTOR_CONTROLLER_CODEGEN_H
#define COAX_TILTROTOR_CONTROLLER_CODEGEN_H

/* Include Files */
#include "rtwtypes.h"
#include "drv_coax_ctrl.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void coax_tiltrotor_controller_codegen(const float x_rb[18],
                                              const float ref_cmd[4],
                                              const DRV_COAX_CTRL_Params *params,
                                              float cmd[4]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for coax_tiltrotor_controller_codegen.h
 *
 * [EOF]
 */
