#ifndef DRV_AIRFRAME_MODEL_H
#define DRV_AIRFRAME_MODEL_H

#define DRV_AIRFRAME_BOARD_MASS_G                 75.0f
#define DRV_AIRFRAME_BATTERY_MASS_G              232.0f
#define DRV_AIRFRAME_BASE_MASS_G                  99.0f
#define DRV_AIRFRAME_SERVO_MOTOR_MASS_G          348.6f

#define DRV_AIRFRAME_BOARD_CG_Z_M                  0.0f
#define DRV_AIRFRAME_BATTERY_CG_Z_M                0.109f
#define DRV_AIRFRAME_BASE_CG_Z_M                  -0.117f
#define DRV_AIRFRAME_SERVO_MOTOR_CG_Z_M           -0.244f

#define DRV_AIRFRAME_MASS_KG                       0.7546f
#define DRV_AIRFRAME_CG_Z_M                       -0.0946f
#define DRV_AIRFRAME_IMU_Z_M                       0.0f

#define DRV_AIRFRAME_TETHER_ATTACH_Z_M             0.1563f
#define DRV_AIRFRAME_TETHER_ATTACH_TO_CG_M         0.2509f
#define DRV_AIRFRAME_TETHER_ROPE_M                 0.6400f
#define DRV_AIRFRAME_TETHER_ROD_TO_CG_M            0.8909f

#define DRV_AIRFRAME_SERVO_DEG_PER_US              0.09f
#define DRV_AIRFRAME_SERVO_US_PER_DEG             11.111111f

#define DRV_AIRFRAME_GRAVITY_M_S2                  9.81f
#define DRV_AIRFRAME_WEIGHT_N                      7.402626f
#define DRV_AIRFRAME_THRUST_TABLE_SCOPE            "dual_motor_total"
#define DRV_AIRFRAME_MAX_TOTAL_THRUST_G         1363.410f
#define DRV_AIRFRAME_MAX_TOTAL_FORCE_N            13.375052f
#define DRV_AIRFRAME_HOVER_THRUST_PERCENT         56.079367f

#endif /* DRV_AIRFRAME_MODEL_H */
