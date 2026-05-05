#include "bsp_bus_servo.h"

#include "usart.h"

#include <stdio.h>
#include <string.h>

#define BSP_BUS_SERVO_MAX_ID          255U
#define BSP_BUS_SERVO_MIN_PULSE_US    500U
#define BSP_BUS_SERVO_MAX_PULSE_US    2500U
#define BSP_BUS_SERVO_MAX_TIME_MS     9999U
#define BSP_BUS_SERVO_MAX_ITEMS       8U
#define BSP_BUS_SERVO_TX_TIMEOUT_MS   100U
#define BSP_BUS_SERVO_MIN_MODE        1U
#define BSP_BUS_SERVO_MAX_MODE        8U
#define BSP_BUS_SERVO_MAX_BAUD_CODE   7U

static BSP_BusServoStatus bus_servo_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return BSP_BUS_SERVO_OK;
    case HAL_TIMEOUT:
        return BSP_BUS_SERVO_TIMEOUT;
    default:
        return BSP_BUS_SERVO_ERROR;
    }
}

static uint8_t bus_servo_is_valid_move(const BSP_BusServoMove *move)
{
    if (move == NULL) {
        return 0U;
    }

    if ((move->pulse_us < BSP_BUS_SERVO_MIN_PULSE_US) ||
        (move->pulse_us > BSP_BUS_SERVO_MAX_PULSE_US)) {
        return 0U;
    }

    return 1U;
}

static BSP_BusServoStatus bus_servo_send_id_command(uint8_t id, const char *suffix)
{
    char command[24];
    int written;

    if (suffix == NULL) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    written = snprintf(command, sizeof(command), "#%03uP%s!", (unsigned int)id, suffix);
    if ((written < 0) || ((uint32_t)written >= sizeof(command))) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    return BSP_BusServo_SendRaw(command);
}

BSP_BusServoStatus BSP_BusServo_SendRaw(const char *command)
{
    size_t length;

    if (command == NULL) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    length = strlen(command);
    if (length == 0U) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    return bus_servo_from_hal(HAL_UART_Transmit(&huart7,
                                                (uint8_t *)command,
                                                (uint16_t)length,
                                                BSP_BUS_SERVO_TX_TIMEOUT_MS));
}

BSP_BusServoStatus BSP_BusServo_Move(uint8_t id,
                                     uint16_t pulse_us,
                                     uint16_t time_ms)
{
    BSP_BusServoMove move = {
        .id = id,
        .pulse_us = pulse_us,
    };

    return BSP_BusServo_MoveMany(&move, 1U, time_ms);
}

BSP_BusServoStatus BSP_BusServo_MoveMany(const BSP_BusServoMove *moves,
                                         uint8_t count,
                                         uint16_t time_ms)
{
    char command[128];
    int written;
    uint32_t used = 0U;

    if ((moves == NULL) || (count == 0U) || (count > BSP_BUS_SERVO_MAX_ITEMS) ||
        (time_ms > BSP_BUS_SERVO_MAX_TIME_MS)) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    command[used++] = '{';

    for (uint8_t index = 0U; index < count; ++index) {
        if (bus_servo_is_valid_move(&moves[index]) == 0U) {
            return BSP_BUS_SERVO_INVALID_PARAM;
        }

        written = snprintf(&command[used],
                           sizeof(command) - used,
                           "#%03uP%04uT%04u!",
                           (unsigned int)moves[index].id,
                           (unsigned int)moves[index].pulse_us,
                           (unsigned int)time_ms);
        if ((written < 0) || ((uint32_t)written >= (sizeof(command) - used))) {
            return BSP_BUS_SERVO_INVALID_PARAM;
        }

        used += (uint32_t)written;
    }

    if ((used + 2U) > sizeof(command)) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    command[used++] = '}';
    command[used] = '\0';

    return BSP_BusServo_SendRaw(command);
}

BSP_BusServoStatus BSP_BusServo_ReadVersion(uint8_t id)
{
    return bus_servo_send_id_command(id, "VER");
}

BSP_BusServoStatus BSP_BusServo_ReadId(uint8_t id)
{
    return bus_servo_send_id_command(id, "ID");
}

BSP_BusServoStatus BSP_BusServo_SetId(uint8_t old_id, uint8_t new_id)
{
    char suffix[8];
    int written;

    written = snprintf(suffix, sizeof(suffix), "ID%03u", (unsigned int)new_id);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    return bus_servo_send_id_command(old_id, suffix);
}

BSP_BusServoStatus BSP_BusServo_ReadPosition(uint8_t id)
{
    return bus_servo_send_id_command(id, "RAD");
}

BSP_BusServoStatus BSP_BusServo_SetMode(uint8_t id, uint8_t mode)
{
    char suffix[6];
    int written;

    if ((mode < BSP_BUS_SERVO_MIN_MODE) || (mode > BSP_BUS_SERVO_MAX_MODE)) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    written = snprintf(suffix, sizeof(suffix), "MOD%u", (unsigned int)mode);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    return bus_servo_send_id_command(id, suffix);
}

BSP_BusServoStatus BSP_BusServo_ReadMode(uint8_t id)
{
    return bus_servo_send_id_command(id, "MOD");
}

BSP_BusServoStatus BSP_BusServo_ReleaseTorque(uint8_t id)
{
    return bus_servo_send_id_command(id, "ULK");
}

BSP_BusServoStatus BSP_BusServo_RestoreTorque(uint8_t id)
{
    return bus_servo_send_id_command(id, "ULR");
}

BSP_BusServoStatus BSP_BusServo_Pause(uint8_t id)
{
    return bus_servo_send_id_command(id, "DPT");
}

BSP_BusServoStatus BSP_BusServo_Continue(uint8_t id)
{
    return bus_servo_send_id_command(id, "DCT");
}

BSP_BusServoStatus BSP_BusServo_Stop(uint8_t id)
{
    return bus_servo_send_id_command(id, "DST");
}

BSP_BusServoStatus BSP_BusServo_SetBaud(uint8_t id, uint8_t baud_code)
{
    char suffix[6];
    int written;

    if (baud_code > BSP_BUS_SERVO_MAX_BAUD_CODE) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    written = snprintf(suffix, sizeof(suffix), "BD%u", (unsigned int)baud_code);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return BSP_BUS_SERVO_INVALID_PARAM;
    }

    return bus_servo_send_id_command(id, suffix);
}

BSP_BusServoStatus BSP_BusServo_SaveCenter(uint8_t id)
{
    return bus_servo_send_id_command(id, "SCK");
}

BSP_BusServoStatus BSP_BusServo_SetStartupPosition(uint8_t id)
{
    return bus_servo_send_id_command(id, "CSD");
}

BSP_BusServoStatus BSP_BusServo_ClearStartupPosition(uint8_t id)
{
    return bus_servo_send_id_command(id, "CSM");
}

BSP_BusServoStatus BSP_BusServo_RestoreStartupPosition(uint8_t id)
{
    return bus_servo_send_id_command(id, "CSR");
}

BSP_BusServoStatus BSP_BusServo_SetMinPosition(uint8_t id)
{
    return bus_servo_send_id_command(id, "SMI");
}

BSP_BusServoStatus BSP_BusServo_SetMaxPosition(uint8_t id)
{
    return bus_servo_send_id_command(id, "SMX");
}

BSP_BusServoStatus BSP_BusServo_FactoryResetKeepId(uint8_t id)
{
    return bus_servo_send_id_command(id, "CLEO");
}

BSP_BusServoStatus BSP_BusServo_FactoryResetFull(uint8_t id)
{
    return bus_servo_send_id_command(id, "CLE");
}

BSP_BusServoStatus BSP_BusServo_RunDemoStep(void)
{
    static uint8_t direction;
    BSP_BusServoMove moves[2];

    if (direction == 0U) {
        moves[0].id = 1U;
        moves[0].pulse_us = 500U;
        moves[1].id = 2U;
        moves[1].pulse_us = 2500U;
        direction = 1U;
    } else {
        moves[0].id = 1U;
        moves[0].pulse_us = 2500U;
        moves[1].id = 2U;
        moves[1].pulse_us = 500U;
        direction = 0U;
    }

    return BSP_BusServo_MoveMany(moves, 2U, 1000U);
}
