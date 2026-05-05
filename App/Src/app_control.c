#include "app_control.h"

#include "app_baro.h"
#include "app_flash.h"
#include "app_imu.h"
#include "app_messages.h"
#include "app_tasks.h"
#include "bsp_bus_servo.h"
#include "bsp_flash.h"
#include "bsp_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_CONTROL_CFG_MAGIC       0x44524346UL
#define APP_CONTROL_CFG_VERSION     1U
#define APP_CONTROL_CFG_ADDRESS     (BSP_GD25Q32_FLASH_SIZE_BYTES - 4096UL)
#define APP_CONTROL_MAX_LINE        128U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    uint32_t checksum;
} APP_ControlFlashRecord;

static APP_ControlConfig control_config;
static uint32_t control_last_heartbeat_ms;
static uint8_t control_reported_hw_once;

static uint32_t app_control_checksum(const uint8_t *data, uint32_t length)
{
    uint32_t sum = 0xA5A55A5AUL;

    for (uint32_t index = 0U; index < length; ++index) {
        sum = (sum << 5U) | (sum >> 27U);
        sum ^= data[index];
        sum += 0x9E3779B9UL;
    }

    return sum;
}

static void app_control_queue_text(const char *format, ...)
{
    APP_UART_TxMessage tx_message;
    va_list args;
    int written;

    if (format == NULL) {
        return;
    }

    va_start(args, format);
    written = vsnprintf(tx_message.text, sizeof(tx_message.text), format, args);
    va_end(args);

    if (written < 0) {
        return;
    }

    if ((uint32_t)written >= sizeof(tx_message.text)) {
        tx_message.length = (uint16_t)(sizeof(tx_message.text) - 1U);
        tx_message.text[tx_message.length] = '\0';
    } else {
        tx_message.length = (uint16_t)written;
    }

    /* Control-plane traffic is generated inside UARTTask itself.
       Send it directly so heartbeats/ACK/status replies do not get
       starved by the shared telemetry queue. */
    (void)BSP_UART_Transmit_USART1((const uint8_t *)tx_message.text,
                                   tx_message.length,
                                   100U);
}

static void app_control_defaults(APP_ControlConfig *config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->servo[0].id = 1U;
    config->servo[0].pulse_us = 1500U;
    config->servo[0].time_ms = 500U;
    config->servo[0].mode = 1U;
    config->servo[0].enabled = 1U;
    config->servo[1].id = 2U;
    config->servo[1].pulse_us = 1500U;
    config->servo[1].time_ms = 500U;
    config->servo[1].mode = 1U;
    config->servo[1].enabled = 1U;
}

static uint8_t app_control_valid_servo_index(uint32_t index)
{
    return (index < APP_CONTROL_SERVO_COUNT) ? 1U : 0U;
}

static uint8_t app_control_parse_u32(const char *text, uint32_t *value)
{
    char *end_ptr;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0U;
    }

    parsed = strtoul(text, &end_ptr, 10);
    if ((end_ptr == text) || (*end_ptr != '\0')) {
        return 0U;
    }

    *value = (uint32_t)parsed;
    return 1U;
}

static void app_control_report_config(void)
{
    app_control_queue_text("CFG loaded=%u valid=%u flash_st=%u\r\n",
                           (unsigned int)control_config.loaded_from_flash,
                           (unsigned int)control_config.flash_valid,
                           (unsigned int)control_config.last_flash_status);
    for (uint32_t index = 0U; index < APP_CONTROL_SERVO_COUNT; ++index) {
        const APP_ControlServoConfig *servo = &control_config.servo[index];

        app_control_queue_text("CFG servo%lu id=%u pulse=%u time=%u mode=%u en=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)servo->id,
                               (unsigned int)servo->pulse_us,
                               (unsigned int)servo->time_ms,
                               (unsigned int)servo->mode,
                               (unsigned int)servo->enabled);
    }
}

static void app_control_report_status(void)
{
    APP_Flash_Status flash_status;
    APP_Baro_Status baro_status;
    APP_IMU_Status imu_status;

    APP_Flash_GetStatus(&flash_status);
    APP_Baro_GetStatus(&baro_status);
    APP_IMU_GetStatus(&imu_status);

    app_control_queue_text("HW FLASH ok=%u stage=%s probe=%ld sr=%ld read=%ld id=%02X%02X%02X exp=C84016 sr1=%02X\r\n",
                           (flash_status.probe_status == 0) &&
                           (flash_status.status1_status == 0) &&
                           (flash_status.read_status == 0),
                           (flash_status.probe_status != 0) ? "probe" :
                           ((flash_status.status1_status != 0) ? "status" :
                           ((flash_status.read_status != 0) ? "read" : "ready")),
                           (long)flash_status.probe_status,
                           (long)flash_status.status1_status,
                           (long)flash_status.read_status,
                           (unsigned int)flash_status.manufacturer_id,
                           (unsigned int)flash_status.memory_type,
                           (unsigned int)flash_status.capacity_id,
                           (unsigned int)flash_status.status1);
    app_control_queue_text("HW SPL06 ok=%u stage=%s init=%ld split=%ld txrx=%ld id=%02X split_id=%02X txrx_id=%02X exp=10 cs=%u miso=%u\r\n",
                           (baro_status.init_status == 0),
                           (baro_status.init_status == 0) ? "ready" : "init",
                           (long)baro_status.init_status,
                           (long)baro_status.split_status,
                           (long)baro_status.txrx_status,
                           (unsigned int)baro_status.product_id,
                           (unsigned int)baro_status.split_id,
                           (unsigned int)baro_status.txrx_id,
                           (unsigned int)baro_status.cs_level,
                           (unsigned int)baro_status.miso_level);
    app_control_queue_text("HW ICM42688 ok=%u stage=%s st=%ld who=%02X exp=47 n=%lu\r\n",
                           (unsigned int)imu_status.initialized,
                           (imu_status.initialized != 0U) ? "ready" : "init",
                           (long)imu_status.last_status,
                           (unsigned int)imu_status.who_am_i,
                           (unsigned long)imu_status.sample_count);

    app_control_queue_text("STATUS flash probe=%ld sr_st=%ld read=%ld id=%02X%02X%02X sr1=%02X\r\n",
                           (long)flash_status.probe_status,
                           (long)flash_status.status1_status,
                           (long)flash_status.read_status,
                           (unsigned int)flash_status.manufacturer_id,
                           (unsigned int)flash_status.memory_type,
                           (unsigned int)flash_status.capacity_id,
                           (unsigned int)flash_status.status1);
    app_control_queue_text("STATUS baro init=%ld split=%ld txrx=%ld id=0x%02X split_id=0x%02X txrx_id=0x%02X bmp=0x%02X cs=%u miso=%u\r\n",
                           (long)baro_status.init_status,
                           (long)baro_status.split_status,
                           (long)baro_status.txrx_status,
                           (unsigned int)baro_status.product_id,
                           (unsigned int)baro_status.split_id,
                           (unsigned int)baro_status.txrx_id,
                           (unsigned int)baro_status.bmp280_id,
                           (unsigned int)baro_status.cs_level,
                           (unsigned int)baro_status.miso_level);
    app_control_queue_text("STATUS imu init=%u st=%ld who=0x%02X n=%lu ax=%d ay=%d az=%d gx=%ld gy=%ld gz=%ld t=%d\r\n",
                           (unsigned int)imu_status.initialized,
                           (long)imu_status.last_status,
                           (unsigned int)imu_status.who_am_i,
                           (unsigned long)imu_status.sample_count,
                           (int)imu_status.accel_x_mg,
                           (int)imu_status.accel_y_mg,
                           (int)imu_status.accel_z_mg,
                           (long)imu_status.gyro_x_mdps,
                           (long)imu_status.gyro_y_mdps,
                           (long)imu_status.gyro_z_mdps,
                           (int)imu_status.temperature_cdeg);
}

static void app_control_report_uart_stats(uint32_t rx_bytes,
                                          uint32_t rx_lines,
                                          uint32_t rx_overflows,
                                          uint32_t rx_errors)
{
    app_control_queue_text("UART1 rx_bytes=%lu rx_lines=%lu rx_overflows=%lu rx_errors=%lu\r\n",
                           (unsigned long)rx_bytes,
                           (unsigned long)rx_lines,
                           (unsigned long)rx_overflows,
                           (unsigned long)rx_errors);
}

static BSP_GD25Q32_Status app_control_load_config(void)
{
    APP_ControlFlashRecord record;
    BSP_GD25Q32_Status status;
    uint32_t checksum;

    status = BSP_FLASH_ReadData(APP_CONTROL_CFG_ADDRESS,
                                (uint8_t *)&record,
                                sizeof(record));
    if (status != BSP_GD25Q32_OK) {
        return status;
    }

    if ((record.magic != APP_CONTROL_CFG_MAGIC) ||
        (record.version != APP_CONTROL_CFG_VERSION) ||
        (record.size != sizeof(record.config))) {
        return BSP_GD25Q32_BAD_ID;
    }

    checksum = app_control_checksum((const uint8_t *)&record.config,
                                    sizeof(record.config));
    if (checksum != record.checksum) {
        return BSP_GD25Q32_ERROR;
    }

    control_config = record.config;
    control_config.loaded_from_flash = 1U;
    control_config.flash_valid = 1U;
    return BSP_GD25Q32_OK;
}

static BSP_GD25Q32_Status app_control_save_config(void)
{
    APP_ControlFlashRecord record;
    BSP_GD25Q32_Status status;

    memset(&record, 0xFF, sizeof(record));
    record.magic = APP_CONTROL_CFG_MAGIC;
    record.version = APP_CONTROL_CFG_VERSION;
    record.size = sizeof(record.config);
    record.config = control_config;
    record.config.loaded_from_flash = 1U;
    record.config.flash_valid = 1U;
    record.checksum = app_control_checksum((const uint8_t *)&record.config,
                                           sizeof(record.config));

    status = BSP_FLASH_EraseSector(APP_CONTROL_CFG_ADDRESS);
    if (status != BSP_GD25Q32_OK) {
        return status;
    }

    return BSP_FLASH_WriteData(APP_CONTROL_CFG_ADDRESS,
                               (const uint8_t *)&record,
                               sizeof(record));
}

static void app_control_servo_move_configured(void)
{
    BSP_BusServoMove moves[APP_CONTROL_SERVO_COUNT];
    uint8_t count = 0U;
    uint16_t time_ms = control_config.servo[0].time_ms;
    BSP_BusServoStatus status;

    for (uint32_t index = 0U; index < APP_CONTROL_SERVO_COUNT; ++index) {
        if (control_config.servo[index].enabled == 0U) {
            continue;
        }

        moves[count].id = control_config.servo[index].id;
        moves[count].pulse_us = control_config.servo[index].pulse_us;
        if (control_config.servo[index].time_ms > time_ms) {
            time_ms = control_config.servo[index].time_ms;
        }
        ++count;
    }

    if (count == 0U) {
        app_control_queue_text("ERR servo no enabled channels\r\n");
        return;
    }

    status = BSP_BusServo_MoveMany(moves, count, time_ms);
    app_control_queue_text("OK servo move_all st=%u count=%u time=%u\r\n",
                           (unsigned int)status,
                           (unsigned int)count,
                           (unsigned int)time_ms);
}

static void app_control_handle_servo(char **tokens, uint32_t count)
{
    uint32_t index;
    uint32_t value;
    BSP_BusServoStatus status;

    if (count < 2U) {
        app_control_queue_text("ERR servo missing subcmd\r\n");
        return;
    }

    if (strcmp(tokens[1], "MOVE") == 0) {
        uint32_t pulse;
        uint32_t time_ms;
        if ((count < 5U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &pulse) == 0U) ||
            (app_control_parse_u32(tokens[4], &time_ms) == 0U) ||
            (app_control_valid_servo_index(index) == 0U)) {
            app_control_queue_text("ERR usage SERVO MOVE index pulse time\r\n");
            return;
        }

        control_config.servo[index].pulse_us = (uint16_t)pulse;
        control_config.servo[index].time_ms = (uint16_t)time_ms;
        status = BSP_BusServo_Move(control_config.servo[index].id,
                                   control_config.servo[index].pulse_us,
                                   control_config.servo[index].time_ms);
        app_control_queue_text("OK servo%lu move st=%u id=%u pulse=%u time=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)status,
                               (unsigned int)control_config.servo[index].id,
                               (unsigned int)control_config.servo[index].pulse_us,
                               (unsigned int)control_config.servo[index].time_ms);
        return;
    }

    if (strcmp(tokens[1], "MOVEALL") == 0) {
        app_control_servo_move_configured();
        return;
    }

    if (strcmp(tokens[1], "ID") == 0) {
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &value) == 0U) ||
            (app_control_valid_servo_index(index) == 0U) ||
            (value > 255U)) {
            app_control_queue_text("ERR usage SERVO ID index id\r\n");
            return;
        }

        control_config.servo[index].id = (uint8_t)value;
        app_control_queue_text("OK servo%lu id=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)control_config.servo[index].id);
        return;
    }

    if (strcmp(tokens[1], "SETID") == 0) {
        uint32_t new_id;
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &new_id) == 0U) ||
            (app_control_valid_servo_index(index) == 0U) ||
            (new_id > 255U)) {
            app_control_queue_text("ERR usage SERVO SETID index new_id\r\n");
            return;
        }

        status = BSP_BusServo_SetId(control_config.servo[index].id, (uint8_t)new_id);
        control_config.servo[index].id = (uint8_t)new_id;
        app_control_queue_text("OK servo%lu setid st=%u id=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)status,
                               (unsigned int)new_id);
        return;
    }

    if (strcmp(tokens[1], "MODE") == 0) {
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &value) == 0U) ||
            (app_control_valid_servo_index(index) == 0U)) {
            app_control_queue_text("ERR usage SERVO MODE index mode\r\n");
            return;
        }

        status = BSP_BusServo_SetMode(control_config.servo[index].id, (uint8_t)value);
        if (status == BSP_BUS_SERVO_OK) {
            control_config.servo[index].mode = (uint8_t)value;
        }
        app_control_queue_text("OK servo%lu mode st=%u mode=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)status,
                               (unsigned int)control_config.servo[index].mode);
        return;
    }

    if (strcmp(tokens[1], "ENABLE") == 0) {
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_parse_u32(tokens[3], &value) == 0U) ||
            (app_control_valid_servo_index(index) == 0U)) {
            app_control_queue_text("ERR usage SERVO ENABLE index 0|1\r\n");
            return;
        }

        control_config.servo[index].enabled = (value != 0U) ? 1U : 0U;
        app_control_queue_text("OK servo%lu enabled=%u\r\n",
                               (unsigned long)index,
                               (unsigned int)control_config.servo[index].enabled);
        return;
    }

    if (strcmp(tokens[1], "CMD") == 0) {
        if ((count < 4U) ||
            (app_control_parse_u32(tokens[2], &index) == 0U) ||
            (app_control_valid_servo_index(index) == 0U)) {
            app_control_queue_text("ERR usage SERVO CMD index action\r\n");
            return;
        }

        if (strcmp(tokens[3], "VER") == 0) {
            status = BSP_BusServo_ReadVersion(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "PID") == 0) {
            status = BSP_BusServo_ReadId(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "RAD") == 0) {
            status = BSP_BusServo_ReadPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "MOD?") == 0) {
            status = BSP_BusServo_ReadMode(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "ULK") == 0) {
            status = BSP_BusServo_ReleaseTorque(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "ULR") == 0) {
            status = BSP_BusServo_RestoreTorque(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "DPT") == 0) {
            status = BSP_BusServo_Pause(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "DCT") == 0) {
            status = BSP_BusServo_Continue(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "DST") == 0) {
            status = BSP_BusServo_Stop(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "BD") == 0) {
            uint32_t baud_code;
            if ((count < 5U) || (app_control_parse_u32(tokens[4], &baud_code) == 0U)) {
                app_control_queue_text("ERR usage SERVO CMD index BD code\r\n");
                return;
            }
            status = BSP_BusServo_SetBaud(control_config.servo[index].id, (uint8_t)baud_code);
        } else if (strcmp(tokens[3], "SCK") == 0) {
            status = BSP_BusServo_SaveCenter(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "CSD") == 0) {
            status = BSP_BusServo_SetStartupPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "CSM") == 0) {
            status = BSP_BusServo_ClearStartupPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "CSR") == 0) {
            status = BSP_BusServo_RestoreStartupPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "SMI") == 0) {
            status = BSP_BusServo_SetMinPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "SMX") == 0) {
            status = BSP_BusServo_SetMaxPosition(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "CLEO") == 0) {
            status = BSP_BusServo_FactoryResetKeepId(control_config.servo[index].id);
        } else if (strcmp(tokens[3], "CLE") == 0) {
            status = BSP_BusServo_FactoryResetFull(control_config.servo[index].id);
        } else {
            app_control_queue_text("ERR unknown servo action %s\r\n", tokens[3]);
            return;
        }

        app_control_queue_text("OK servo%lu cmd=%s st=%u\r\n",
                               (unsigned long)index,
                               tokens[3],
                               (unsigned int)status);
        return;
    }

    if (strcmp(tokens[1], "RAW") == 0) {
        if (count < 3U) {
            app_control_queue_text("ERR usage SERVO RAW command\r\n");
            return;
        }

        status = BSP_BusServo_SendRaw(tokens[2]);
        app_control_queue_text("OK servo raw st=%u\r\n", (unsigned int)status);
        return;
    }

    app_control_queue_text("ERR unknown servo subcmd %s\r\n", tokens[1]);
}

void APP_Control_Init(void)
{
    BSP_GD25Q32_Status load_status;

    app_control_defaults(&control_config);
    load_status = app_control_load_config();
    control_config.last_flash_status = (uint8_t)load_status;
    if (load_status != BSP_GD25Q32_OK) {
        control_config.loaded_from_flash = 0U;
        control_config.flash_valid = 0U;
    }

    app_control_queue_text("READY drone-H743 tcp-control servo_slots=2 cfg_loaded=%u cfg_valid=%u\r\n",
                           (unsigned int)control_config.loaded_from_flash,
                           (unsigned int)control_config.flash_valid);
}

void APP_Control_Tick(void)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - control_last_heartbeat_ms) < 2000U) {
        return;
    }

    control_last_heartbeat_ms = now_ms;
    app_control_queue_text("READY ms=%lu servo0_id=%u servo1_id=%u cfg_valid=%u\r\n",
                           (unsigned long)now_ms,
                           (unsigned int)control_config.servo[0].id,
                           (unsigned int)control_config.servo[1].id,
                           (unsigned int)control_config.flash_valid);

    if (control_reported_hw_once == 0U) {
        control_reported_hw_once = 1U;
        app_control_report_status();
    }
}

void APP_Control_ProcessLine(const char *line)
{
    char buffer[APP_CONTROL_MAX_LINE];
    char *tokens[8];
    uint32_t count = 0U;
    char *token;

    if ((line == NULL) || (*line == '\0')) {
        return;
    }

    app_control_queue_text("RX %s\r\n", line);

    (void)snprintf(buffer, sizeof(buffer), "%s", line);
    token = strtok(buffer, " \t\r\n");
    while ((token != NULL) && (count < (sizeof(tokens) / sizeof(tokens[0])))) {
        tokens[count++] = token;
        token = strtok(NULL, " \t\r\n");
    }

    if (count == 0U) {
        return;
    }

    app_control_queue_text("ACK %s\r\n", tokens[0]);

    if (strcmp(tokens[0], "PING") == 0) {
        app_control_queue_text("PONG drone-H743\r\n");
    } else if (strcmp(tokens[0], "STATUS?") == 0) {
        app_control_report_status();
    } else if (strcmp(tokens[0], "CONFIG?") == 0) {
        app_control_report_config();
    } else if (strcmp(tokens[0], "SAVE") == 0) {
        BSP_GD25Q32_Status save_status = app_control_save_config();
        control_config.last_flash_status = (uint8_t)save_status;
        control_config.flash_valid = (save_status == BSP_GD25Q32_OK) ? 1U : control_config.flash_valid;
        app_control_queue_text("OK save st=%u\r\n", (unsigned int)save_status);
    } else if (strcmp(tokens[0], "LOAD") == 0) {
        BSP_GD25Q32_Status load_status = app_control_load_config();
        control_config.last_flash_status = (uint8_t)load_status;
        app_control_queue_text("OK load st=%u\r\n", (unsigned int)load_status);
    } else if (strcmp(tokens[0], "DEFAULTS") == 0) {
        app_control_defaults(&control_config);
        app_control_queue_text("OK defaults\r\n");
    } else if (strcmp(tokens[0], "SERVO") == 0) {
        app_control_handle_servo(tokens, count);
    } else {
        app_control_queue_text("ERR unknown cmd %s\r\n", tokens[0]);
    }
}

void APP_Control_GetConfig(APP_ControlConfig *config)
{
    if (config == NULL) {
        return;
    }

    *config = control_config;
}

void APP_Control_ReportUartStats(uint32_t rx_bytes,
                                  uint32_t rx_lines,
                                  uint32_t rx_overflows,
                                  uint32_t rx_errors)
{
    app_control_report_uart_stats(rx_bytes, rx_lines, rx_overflows, rx_errors);
}
