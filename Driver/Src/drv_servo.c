#include "drv_servo.h"

#include "bsp_cache.h"

#include <stdio.h>
#include <string.h>

#define DRV_SERVO_MAX_BAUDRATE     1000000U
#define DRV_SERVO_MIN_BAUDRATE     1200U
#define DRV_SERVO_MAX_ID           255U
#define DRV_SERVO_MAX_ITEMS       8U
#define DRV_SERVO_TX_TIMEOUT_MS   100U
#define DRV_SERVO_MIN_MODE        1U
#define DRV_SERVO_MAX_MODE        8U
#define DRV_SERVO_MAX_BAUD_CODE   7U
#define DRV_SERVO_COMMAND_BUFFER_SIZE 128U
#define DRV_SERVO_DMA_MIN_TIMEOUT_MS 10U
#define DRV_SERVO_UART_BITS_PER_BYTE 10U

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t servo_dma_tx_buffer[DRV_SERVO_COMMAND_BUFFER_SIZE];
static volatile DRV_SERVO_Diag servo_diag;
static volatile uint32_t servo_dma_start_tick_ms;
static volatile uint32_t servo_dma_timeout_ms;
static UART_HandleTypeDef * volatile servo_dma_huart;

static uint32_t servo_estimate_dma_timeout_ms(UART_HandleTypeDef *huart,
                                              uint16_t length)
{
    uint32_t baud = ((huart != NULL) && (huart->Init.BaudRate != 0U)) ?
                    huart->Init.BaudRate : 115200U;
    uint32_t bits = (uint32_t)length * DRV_SERVO_UART_BITS_PER_BYTE;
    uint32_t wire_ms = ((bits * 1000U) + baud - 1U) / baud;
    uint32_t timeout_ms = wire_ms + 5U;

    return (timeout_ms < DRV_SERVO_DMA_MIN_TIMEOUT_MS) ?
           DRV_SERVO_DMA_MIN_TIMEOUT_MS : timeout_ms;
}

static void servo_diag_capture_uart(UART_HandleTypeDef *huart)
{
    servo_diag.last_uart_state = (huart != NULL) ? (uint32_t)huart->gState : 0U;
    servo_diag.last_uart_error = (huart != NULL) ? huart->ErrorCode : 0U;
    servo_diag.last_dma_state = ((huart != NULL) && (huart->hdmatx != NULL)) ?
                                (uint32_t)huart->hdmatx->State : 0U;
    servo_diag.last_dma_error = ((huart != NULL) && (huart->hdmatx != NULL)) ?
                                huart->hdmatx->ErrorCode : 0U;
}

static void servo_try_recover_stuck_dma(UART_HandleTypeDef *huart)
{
    uint32_t elapsed_ms;

    if ((huart == NULL) || (servo_dma_start_tick_ms == 0U) ||
        (huart->gState == HAL_UART_STATE_READY)) {
        return;
    }

    elapsed_ms = HAL_GetTick() - servo_dma_start_tick_ms;
    if (elapsed_ms < servo_dma_timeout_ms) {
        return;
    }

    (void)HAL_UART_AbortTransmit(huart);
    servo_dma_start_tick_ms = 0U;
    servo_dma_timeout_ms = 0U;
    servo_dma_huart = NULL;
    servo_diag.tx_recover_count++;
    servo_diag_capture_uart(huart);
}

static DRV_SERVO_Status servo_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:      return DRV_SERVO_OK;
    case HAL_TIMEOUT:  return DRV_SERVO_TIMEOUT;
    default:           return DRV_SERVO_ERROR;
    }
}

static uint8_t servo_is_valid_move(const DRV_SERVO_MoveCmd *move)
{
    if (move == NULL) { return 0U; }
    if ((move->pulse_us < DRV_SERVO_MIN_PULSE_US) ||
        (move->pulse_us > DRV_SERVO_MAX_PULSE_US)) { return 0U; }
    return 1U;
}

static DRV_SERVO_Status servo_build_move_many_command(const DRV_SERVO_MoveCmd *moves,
                                                      uint8_t count,
                                                      uint16_t time_ms,
                                                      char *command,
                                                      uint16_t command_size,
                                                      uint16_t *length)
{
    int written;
    uint32_t used = 0U;

    if ((moves == NULL) || (command == NULL) || (length == NULL) ||
        (count == 0U) || (count > DRV_SERVO_MAX_ITEMS) ||
        (time_ms > DRV_SERVO_MAX_TIME_MS) ||
        (command_size < 3U)) {
        return DRV_SERVO_INVALID_PARAM;
    }

    command[used++] = '{';
    command[used++] = 'G';
    command[used++] = '0';
    command[used++] = '0';
    command[used++] = '0';
    command[used++] = '0';

    for (uint8_t index = 0U; index < count; ++index) {
        if (servo_is_valid_move(&moves[index]) == 0U) {
            return DRV_SERVO_INVALID_PARAM;
        }

        written = snprintf(&command[used], (size_t)command_size - used,
                           "#%03uP%04uT%04u!",
                           (unsigned int)moves[index].id,
                           (unsigned int)moves[index].pulse_us,
                           (unsigned int)time_ms);
        if ((written < 0) || ((uint32_t)written >= ((uint32_t)command_size - used))) {
            return DRV_SERVO_INVALID_PARAM;
        }
        used += (uint32_t)written;
    }

    if ((used + 2U) > command_size) { return DRV_SERVO_INVALID_PARAM; }

    command[used++] = '}';
    command[used] = '\0';
    *length = (uint16_t)used;

    return DRV_SERVO_OK;
}

static DRV_SERVO_Status servo_send_id_command(DRV_SERVO_Device *dev,
                                              uint8_t id, const char *suffix)
{
    char command[24];
    int written;

    if (suffix == NULL) { return DRV_SERVO_INVALID_PARAM; }

    written = snprintf(command, sizeof(command), "#%03uP%s!", (unsigned int)id, suffix);
    if ((written < 0) || ((uint32_t)written >= sizeof(command))) {
        return DRV_SERVO_INVALID_PARAM;
    }

    return DRV_SERVO_SendRaw(dev, command);
}

DRV_SERVO_Status DRV_SERVO_SendRaw(DRV_SERVO_Device *dev, const char *command)
{
    size_t length;
    uint32_t timeout = (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms
                                                    : DRV_SERVO_TX_TIMEOUT_MS;

    if (command == NULL) { return DRV_SERVO_INVALID_PARAM; }

    length = strlen(command);
    if (length == 0U) { return DRV_SERVO_INVALID_PARAM; }

    return servo_from_hal(HAL_UART_Transmit(dev->bus.huart, (uint8_t *)command,
                                            (uint16_t)length, timeout));
}

uint16_t DRV_SERVO_ReadResponse(DRV_SERVO_Device *dev, char *buf, uint16_t max_len)
{
    uint16_t count = 0U;
    uint32_t byte_timeout = 10U;

    if ((buf == NULL) || (max_len == 0U)) { return 0U; }

    HAL_HalfDuplex_EnableReceiver(dev->bus.huart);

    while (count < max_len) {
        HAL_StatusTypeDef status;
        status = HAL_UART_Receive(dev->bus.huart, (uint8_t *)&buf[count], 1U,
                                  byte_timeout);
        if (status != HAL_OK) { break; }
        count++;
    }

    HAL_HalfDuplex_EnableTransmitter(dev->bus.huart);
    return count;
}

uint32_t DRV_SERVO_GetBaudRate(const DRV_SERVO_Device *dev)
{
    return dev->bus.huart->Init.BaudRate;
}

DRV_SERVO_Status DRV_SERVO_SetBaudRate(DRV_SERVO_Device *dev, uint32_t baud_rate)
{
    UART_HandleTypeDef *huart = dev->bus.huart;

    if ((baud_rate < DRV_SERVO_MIN_BAUDRATE) || (baud_rate > DRV_SERVO_MAX_BAUDRATE)) {
        return DRV_SERVO_INVALID_PARAM;
    }

    huart->Init.BaudRate = baud_rate;
    return servo_from_hal(HAL_HalfDuplex_Init(huart));
}

uint16_t DRV_SERVO_PositionToPulse(uint16_t position)
{
    uint32_t span = DRV_SERVO_MAX_PULSE_US - DRV_SERVO_MIN_PULSE_US;

    if (position > DRV_SERVO_POSITION_MAX) {
        position = DRV_SERVO_POSITION_MAX;
    }

    return (uint16_t)(DRV_SERVO_MIN_PULSE_US +
                      (((uint32_t)position * span) / DRV_SERVO_POSITION_MAX));
}

DRV_SERVO_Status DRV_SERVO_Move(DRV_SERVO_Device *dev, uint8_t id,
                                uint16_t pulse_us, uint16_t time_ms)
{
    char command[32];
    int written;

    if ((pulse_us < DRV_SERVO_MIN_PULSE_US) || (pulse_us > DRV_SERVO_MAX_PULSE_US) ||
        (time_ms > DRV_SERVO_MAX_TIME_MS)) {
        return DRV_SERVO_INVALID_PARAM;
    }

    written = snprintf(command, sizeof(command), "#%03uP%04uT%04u!",
                       (unsigned int)id, (unsigned int)pulse_us, (unsigned int)time_ms);
    if ((written < 0) || ((uint32_t)written >= sizeof(command))) {
        return DRV_SERVO_INVALID_PARAM;
    }

    return DRV_SERVO_SendRaw(dev, command);
}

DRV_SERVO_Status DRV_SERVO_MovePosition(DRV_SERVO_Device *dev, uint8_t id,
                                        uint16_t position, uint16_t time_ms)
{
    if (position > DRV_SERVO_POSITION_MAX) {
        return DRV_SERVO_INVALID_PARAM;
    }

    return DRV_SERVO_Move(dev, id, DRV_SERVO_PositionToPulse(position), time_ms);
}

DRV_SERVO_Status DRV_SERVO_MoveMany(DRV_SERVO_Device *dev,
                                    const DRV_SERVO_MoveCmd *moves,
                                    uint8_t count, uint16_t time_ms)
{
    char command[DRV_SERVO_COMMAND_BUFFER_SIZE];
    uint16_t length = 0U;
    DRV_SERVO_Status status;

    status = servo_build_move_many_command(moves, count, time_ms,
                                           command, (uint16_t)sizeof(command),
                                           &length);
    if (status != DRV_SERVO_OK) {
        return status;
    }
    return DRV_SERVO_SendRaw(dev, command);
}

DRV_SERVO_Status DRV_SERVO_MoveManyAsync(DRV_SERVO_Device *dev,
                                         const DRV_SERVO_MoveCmd *moves,
                                         uint8_t count, uint16_t time_ms)
{
    uint16_t length = 0U;
    DRV_SERVO_Status status;
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (dev->bus.huart == NULL)) {
        return DRV_SERVO_INVALID_PARAM;
    }

    if (dev->bus.huart->hdmatx == NULL) {
        servo_diag.last_status = DRV_SERVO_ERROR;
        servo_diag_capture_uart(dev->bus.huart);
        return DRV_SERVO_ERROR;
    }

    servo_try_recover_stuck_dma(dev->bus.huart);
    if (dev->bus.huart->gState != HAL_UART_STATE_READY) {
        servo_diag.tx_busy_count++;
        servo_diag.last_status = DRV_SERVO_BUSY;
        servo_diag_capture_uart(dev->bus.huart);
        return DRV_SERVO_BUSY;
    }

    status = servo_build_move_many_command(moves, count, time_ms,
                                           (char *)servo_dma_tx_buffer,
                                           (uint16_t)sizeof(servo_dma_tx_buffer),
                                           &length);
    if (status != DRV_SERVO_OK) {
        servo_diag.last_status = status;
        return status;
    }

    BSP_Cache_CleanDCache(servo_dma_tx_buffer, length);
    hal_status = HAL_UART_Transmit_DMA(dev->bus.huart, servo_dma_tx_buffer, length);

    if (hal_status == HAL_BUSY) {
        servo_diag.tx_busy_count++;
        servo_diag.last_status = DRV_SERVO_BUSY;
        servo_diag_capture_uart(dev->bus.huart);
        return DRV_SERVO_BUSY;
    }

    status = servo_from_hal(hal_status);
    servo_diag.last_status = status;
    servo_diag_capture_uart(dev->bus.huart);

    if (status == DRV_SERVO_OK) {
        servo_diag.tx_start_count++;
        servo_diag.last_length = length;
        servo_dma_timeout_ms = servo_estimate_dma_timeout_ms(dev->bus.huart, length);
        servo_dma_start_tick_ms = HAL_GetTick();
        servo_dma_huart = dev->bus.huart;
    } else {
        servo_diag.tx_error_count++;
    }

    return status;
}

void DRV_SERVO_GetDiag(DRV_SERVO_Diag *diag)
{
    if (diag == NULL) { return; }
    *diag = servo_diag;
}

void DRV_SERVO_OnUartTxComplete(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart != servo_dma_huart)) {
        return;
    }

    servo_dma_start_tick_ms = 0U;
    servo_dma_timeout_ms = 0U;
    servo_dma_huart = NULL;
    servo_diag.tx_complete_count++;
    servo_diag_capture_uart(huart);
}

void DRV_SERVO_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart != servo_dma_huart)) {
        return;
    }

    servo_dma_start_tick_ms = 0U;
    servo_dma_timeout_ms = 0U;
    servo_dma_huart = NULL;
    servo_diag.tx_error_count++;
    servo_diag.last_status = DRV_SERVO_ERROR;
    servo_diag_capture_uart(huart);
}

DRV_SERVO_Status DRV_SERVO_ReadVersion(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "VER"); }

DRV_SERVO_Status DRV_SERVO_ReadId(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "ID"); }

DRV_SERVO_Status DRV_SERVO_SetId(DRV_SERVO_Device *dev, uint8_t old_id, uint8_t new_id)
{
    char suffix[8];
    int written;
    written = snprintf(suffix, sizeof(suffix), "ID%03u", (unsigned int)new_id);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return DRV_SERVO_INVALID_PARAM;
    }
    return servo_send_id_command(dev, old_id, suffix);
}

DRV_SERVO_Status DRV_SERVO_ReadPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "RAD"); }

DRV_SERVO_Status DRV_SERVO_SetMode(DRV_SERVO_Device *dev, uint8_t id, uint8_t mode)
{
    char suffix[6];
    int written;
    if ((mode < DRV_SERVO_MIN_MODE) || (mode > DRV_SERVO_MAX_MODE)) {
        return DRV_SERVO_INVALID_PARAM;
    }
    written = snprintf(suffix, sizeof(suffix), "MOD%u", (unsigned int)mode);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return DRV_SERVO_INVALID_PARAM;
    }
    return servo_send_id_command(dev, id, suffix);
}

DRV_SERVO_Status DRV_SERVO_ReadMode(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "MOD"); }

DRV_SERVO_Status DRV_SERVO_ReleaseTorque(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "ULK"); }

DRV_SERVO_Status DRV_SERVO_RestoreTorque(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "ULR"); }

DRV_SERVO_Status DRV_SERVO_Pause(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "DPT"); }

DRV_SERVO_Status DRV_SERVO_Continue(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "DCT"); }

DRV_SERVO_Status DRV_SERVO_Stop(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "DST"); }

DRV_SERVO_Status DRV_SERVO_SetBaud(DRV_SERVO_Device *dev, uint8_t id, uint8_t baud_code)
{
    char suffix[6];
    int written;
    if (baud_code > DRV_SERVO_MAX_BAUD_CODE) { return DRV_SERVO_INVALID_PARAM; }
    written = snprintf(suffix, sizeof(suffix), "BD%u", (unsigned int)baud_code);
    if ((written < 0) || ((uint32_t)written >= sizeof(suffix))) {
        return DRV_SERVO_INVALID_PARAM;
    }
    return servo_send_id_command(dev, id, suffix);
}

DRV_SERVO_Status DRV_SERVO_SaveCenter(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "SCK"); }

DRV_SERVO_Status DRV_SERVO_SetStartupPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CSD"); }

DRV_SERVO_Status DRV_SERVO_ClearStartupPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CSM"); }

DRV_SERVO_Status DRV_SERVO_RestoreStartupPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CSR"); }

DRV_SERVO_Status DRV_SERVO_SetMinPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "SMI"); }

DRV_SERVO_Status DRV_SERVO_SetMaxPosition(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "SMX"); }

DRV_SERVO_Status DRV_SERVO_FactoryResetKeepId(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CLEO"); }

DRV_SERVO_Status DRV_SERVO_FactoryResetFull(DRV_SERVO_Device *dev, uint8_t id)
{ return servo_send_id_command(dev, id, "CLE"); }
