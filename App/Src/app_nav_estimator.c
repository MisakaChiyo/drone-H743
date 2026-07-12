#include "app_nav_estimator.h"

#include <stddef.h>

static DRV_NAV_EKF_Diagnostics nav_estimator_velocity_ekf;

void APP_NavEstimator_PublishVelocityEKF(
    const DRV_NAV_EKF_Diagnostics *diagnostics)
{
    if (diagnostics == NULL) {
        return;
    }

    nav_estimator_velocity_ekf = *diagnostics;
}

void APP_NavEstimator_GetVelocityEKF(
    DRV_NAV_EKF_Diagnostics *diagnostics)
{
    if (diagnostics == NULL) {
        return;
    }

    *diagnostics = nav_estimator_velocity_ekf;
}
