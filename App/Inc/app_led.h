#ifndef APP_LED_H
#define APP_LED_H

#include <stdint.h>

typedef enum {
    APP_LED_ARM_BLOCK_NONE = 0,
    APP_LED_ARM_BLOCK_NO_RC = 1,
    APP_LED_ARM_BLOCK_RC_LOSS = 2,
    APP_LED_ARM_BLOCK_ARM_SWITCH = 3,
    APP_LED_ARM_BLOCK_THROTTLE_HIGH = 4,
    APP_LED_ARM_BLOCK_IMU = 5,
} APP_LED_ArmBlockReason;

typedef enum {
    APP_LED_SERVO_CAL_NONE = 0,
    APP_LED_SERVO_CAL_RELEASED = 1,
    APP_LED_SERVO_CAL_SAVE_ACK = 2,
    APP_LED_SERVO_CAL_ERROR = 3,
} APP_LED_ServoCalMode;

void APP_LED_Task_Init(void);
void APP_LED_Task_Step(void);
void APP_LED_SetArmStatus(uint8_t armed, APP_LED_ArmBlockReason reason);
void APP_LED_SetServoCalMode(APP_LED_ServoCalMode mode);

#endif
