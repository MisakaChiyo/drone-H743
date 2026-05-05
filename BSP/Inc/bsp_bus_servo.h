#ifndef BSP_BUS_SERVO_H
#define BSP_BUS_SERVO_H

#include "main.h"

#include <stdint.h>

typedef enum {
    BSP_BUS_SERVO_OK = 0,
    BSP_BUS_SERVO_ERROR,
    BSP_BUS_SERVO_INVALID_PARAM,
    BSP_BUS_SERVO_TIMEOUT
} BSP_BusServoStatus;

typedef struct {
    uint8_t id;
    uint16_t pulse_us;
} BSP_BusServoMove;

BSP_BusServoStatus BSP_BusServo_SendRaw(const char *command);
BSP_BusServoStatus BSP_BusServo_Move(uint8_t id,
                                     uint16_t pulse_us,
                                     uint16_t time_ms);
BSP_BusServoStatus BSP_BusServo_MoveMany(const BSP_BusServoMove *moves,
                                         uint8_t count,
                                         uint16_t time_ms);
BSP_BusServoStatus BSP_BusServo_ReadVersion(uint8_t id);
BSP_BusServoStatus BSP_BusServo_ReadId(uint8_t id);
BSP_BusServoStatus BSP_BusServo_SetId(uint8_t old_id, uint8_t new_id);
BSP_BusServoStatus BSP_BusServo_ReadPosition(uint8_t id);
BSP_BusServoStatus BSP_BusServo_SetMode(uint8_t id, uint8_t mode);
BSP_BusServoStatus BSP_BusServo_ReadMode(uint8_t id);
BSP_BusServoStatus BSP_BusServo_ReleaseTorque(uint8_t id);
BSP_BusServoStatus BSP_BusServo_RestoreTorque(uint8_t id);
BSP_BusServoStatus BSP_BusServo_Pause(uint8_t id);
BSP_BusServoStatus BSP_BusServo_Continue(uint8_t id);
BSP_BusServoStatus BSP_BusServo_Stop(uint8_t id);
BSP_BusServoStatus BSP_BusServo_SetBaud(uint8_t id, uint8_t baud_code);
BSP_BusServoStatus BSP_BusServo_SaveCenter(uint8_t id);
BSP_BusServoStatus BSP_BusServo_SetStartupPosition(uint8_t id);
BSP_BusServoStatus BSP_BusServo_ClearStartupPosition(uint8_t id);
BSP_BusServoStatus BSP_BusServo_RestoreStartupPosition(uint8_t id);
BSP_BusServoStatus BSP_BusServo_SetMinPosition(uint8_t id);
BSP_BusServoStatus BSP_BusServo_SetMaxPosition(uint8_t id);
BSP_BusServoStatus BSP_BusServo_FactoryResetKeepId(uint8_t id);
BSP_BusServoStatus BSP_BusServo_FactoryResetFull(uint8_t id);
BSP_BusServoStatus BSP_BusServo_RunDemoStep(void);

#endif
