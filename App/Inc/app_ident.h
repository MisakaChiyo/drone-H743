#ifndef APP_IDENT_H
#define APP_IDENT_H

#include <stdint.h>

typedef enum {
    APP_IDENT_STATE_IDLE = 0,
    APP_IDENT_STATE_ARMED,
    APP_IDENT_STATE_RUNNING,
    APP_IDENT_STATE_DONE,
    APP_IDENT_STATE_ABORTED
} APP_IdentState;

typedef struct {
    uint32_t now_ms;
    float roll_deg;
    float pitch_deg;
    float gyro_x_dps;
    float gyro_y_dps;
    uint8_t rc_link_ok;
    uint8_t rc_armed;
    uint8_t imu_valid;
    uint16_t throttle_us;
} APP_IdentObserve;

void APP_Ident_Init(void);
APP_IdentState APP_Ident_GetState(void);
uint8_t APP_Ident_IsRunning(void);
void APP_Ident_ReportStatus(void);
uint8_t APP_Ident_Arm(void);
void APP_Ident_Disarm(void);
void APP_Ident_Stop(const char *reason);
uint8_t APP_Ident_SetCenter(uint16_t alpha_us, uint16_t beta_us);
uint8_t APP_Ident_StartStep(const char *axis, int32_t pulse_us, uint32_t duration_ms);
uint8_t APP_Ident_StartDoublet(const char *axis, int32_t pulse_us, uint32_t hold_ms, uint32_t repeat);
uint8_t APP_Ident_StartPrbs(const char *axis,
                            int32_t pulse_us,
                            uint32_t bit_ms,
                            uint32_t duration_ms,
                            uint32_t seed);
uint8_t APP_Ident_ApplyPid(const char *axis, const char *kp_text, const char *kd_text);
void APP_Ident_Update(uint32_t now_ms);
void APP_Ident_GetServoTargets(uint16_t *alpha_us, uint16_t *beta_us);
void APP_Ident_Observe(const APP_IdentObserve *obs);

#endif
