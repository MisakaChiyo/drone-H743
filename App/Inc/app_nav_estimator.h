#ifndef APP_NAV_ESTIMATOR_H
#define APP_NAV_ESTIMATOR_H

#include "drv_nav_ekf.h"

#ifdef __cplusplus
extern "C" {
#endif

void APP_NavEstimator_PublishVelocityEKF(
    const DRV_NAV_EKF_Diagnostics *diagnostics);
void APP_NavEstimator_GetVelocityEKF(
    DRV_NAV_EKF_Diagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif
