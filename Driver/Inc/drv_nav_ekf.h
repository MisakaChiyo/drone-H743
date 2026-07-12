#ifndef DRV_NAV_EKF_H
#define DRV_NAV_EKF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float process_accel_noise_m_s2;
    float bias_random_walk_m_s3;
    float flow_noise_m_s;
    float flow_gate_nis;
    float initial_velocity_variance;
    float initial_bias_variance;
    float max_dt_sec;
    float max_velocity_m_s;
    float predict_leak_hz;
} DRV_NAV_EKF_Config;

typedef struct {
    float vel_m_s[2];
    float accel_bias_m_s2[2];
    float covariance[4][4];
    DRV_NAV_EKF_Config config;
    uint8_t initialized;
    uint32_t predict_count;
    uint32_t flow_update_count;
    uint32_t flow_reject_count;
    uint32_t flow_skip_count;
    uint32_t last_flow_sample_ms;
    uint32_t last_flow_update_ms;
    float last_dt_sec;
    float last_flow_noise_m_s;
    float last_innovation_m_s[2];
    float last_nis;
    float last_gate_nis;
} DRV_NAV_EKF_State;

typedef struct {
    float vel_m_s[2];
    float accel_bias_m_s2[2];
    float covariance_diag[4];
    uint32_t predict_count;
    uint32_t flow_update_count;
    uint32_t flow_reject_count;
    uint32_t flow_skip_count;
    uint32_t last_flow_update_ms;
    float last_dt_sec;
    float last_flow_noise_m_s;
    float last_innovation_m_s[2];
    float last_nis;
    float last_gate_nis;
    uint8_t initialized;
} DRV_NAV_EKF_Diagnostics;

void DRV_NAV_EKF_DefaultConfig(DRV_NAV_EKF_Config *config);
void DRV_NAV_EKF_Reset(DRV_NAV_EKF_State *state,
                       const DRV_NAV_EKF_Config *config);
void DRV_NAV_EKF_Predict(DRV_NAV_EKF_State *state,
                         float acc_x_m_s2,
                         float acc_y_m_s2,
                         float dt_sec);
uint8_t DRV_NAV_EKF_FuseFlow(DRV_NAV_EKF_State *state,
                             float flow_vx_m_s,
                             float flow_vy_m_s,
                             uint8_t flow_valid,
                             uint32_t flow_sample_ms,
                             float flow_noise_m_s);
void DRV_NAV_EKF_GetVelocity(const DRV_NAV_EKF_State *state,
                             float *vx_m_s,
                             float *vy_m_s);
void DRV_NAV_EKF_GetDiagnostics(const DRV_NAV_EKF_State *state,
                                DRV_NAV_EKF_Diagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif
