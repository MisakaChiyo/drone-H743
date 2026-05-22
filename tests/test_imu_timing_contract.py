from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_attitude_filter_uses_fixed_1ms_dt_with_timestamp_option() -> None:
    header = read("App/Inc/app_sensor.h")
    source = read("App/Src/app_sensor.c")
    freertos = read("Core/Src/freertos.c")

    assert "float dt_sec" in header
    assert "APP_IMU_ACCEL_CORRECTION_TAU_SEC" in source
    assert "osKernelGetTickCount()" not in source
    assert "#define STABILIZER_USE_FIXED_IMU_DT    1U" in freertos
    assert "float dt_sec = SENSOR_IMU_DEFAULT_DT_SEC;" in freertos
    assert "#if (STABILIZER_USE_FIXED_IMU_DT == 0U)" in freertos
    assert "msg.base.timestamp_us - last_imu_timestamp_us" in freertos
    assert "APP_IMU_UpdateAttitude(&msg.imu, &roll, &pitch, &yaw,\n                             dt_sec, stabilizer_seq)" in freertos


def test_slow_mag_step_is_not_run_for_every_imu_sample() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "SENSOR_MAG_PERIOD_US" in freertos
    assert "last_mag_step_us" in freertos
    assert "((now_us - last_mag_step_us) >= SENSOR_MAG_PERIOD_US)" in freertos


def test_vofa_stream_reports_imu_rate_and_interrupt_vs_poll_counts() -> None:
    messages = read("App/Inc/app_messages.h")
    freertos = read("Core/Src/freertos.c")

    assert "uint32_t imu_poll_ready_count;" in messages
    assert "uint32_t imu_irq_ready_count = 0U;" in freertos
    assert "uint32_t imu_poll_ready_count = 0U;" in freertos
    assert "imu_irq_ready_count++;" in freertos
    assert "msg.imu_data_ready_count = imu_irq_ready_count;" in freertos
    assert "msg.imu_poll_ready_count = imu_poll_ready_count;" in freertos
    assert "#define VOFA_DATA_SIZE 11U" in freertos
    assert "vofa_data[10] = (float)(SVC_Timestamp_Us() / 1000ULL) * 0.001f;" in freertos


def test_sensor_task_uses_interrupt_with_bounded_ready_fallback() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "SENSOR_IMU_POLL_TIMEOUT_MS" not in freertos
    assert "osThreadFlagsWait(SENSOR_IMU_DATA_READY_FLAG, osFlagsWaitAny,\n                           SENSOR_IMU_DRDY_TIMEOUT_MS)" in freertos
    assert "BSP_IMU_IsDataReady(&imu_ready)" in freertos
    assert "imu_poll_ready_count++;" in freertos
    assert "SENSOR_IMU_DRDY_MISS_FAULT_LIMIT" in freertos
    assert "stabilizer_latch_imu_fault(STABILIZER_IMU_FAULT_DRDY_TIMEOUT);" in freertos


def test_imu_runtime_fault_does_not_auto_reinit_and_can_recover_on_good_sample() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define SENSOR_IMU_DRDY_TIMEOUT_MS" in freertos
    assert "#define SENSOR_IMU_DRDY_MISS_FAULT_LIMIT" in freertos
    assert "#define SENSOR_IMU_READ_FAIL_LIMIT" in freertos
    assert "stabilizer_latch_imu_fault(STABILIZER_IMU_FAULT_DRDY_TIMEOUT);" in freertos
    assert "stabilizer_latch_imu_fault(STABILIZER_IMU_FAULT_READ_FAIL);" in freertos
    assert "BSP_IMU_Invalidate();" in freertos
    assert freertos.count("BSP_IMU_Init()") == 1
    assert freertos.count("BSP_IMU_Invalidate();") == 1
    assert "stabilizer_clear_imu_fault();" in freertos


def test_stabilizer_holds_last_servo_target_on_imu_dropout_after_first_sample() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_IMU_STALE_MS" in freertos
    assert "#define STABILIZER_IMU_FAILSAFE_MS" not in freertos
    assert "static volatile uint8_t stabilizer_imu_fault_latched = 0U;" in freertos
    assert "uint8_t imu_control_valid = 0U;" in freertos
    assert "((now - stabilizer_imu_last_sample_ms) <= STABILIZER_IMU_STALE_MS)" in freertos
    assert "if (imu_control_valid != 0U)" in freertos
    assert "moves[0].pulse_us = stabilizer_latest_servo_target_us[0];" in freertos
    assert "moves[1].pulse_us = stabilizer_latest_servo_target_us[1];" in freertos
    assert "else if (has_imu_sample == 0U)" in freertos
    assert "运行中 IMU 异常保持上一目标" in freertos
    assert "moves[0].pulse_us = DRV_COAX_CTRL_SERVO_CENTER_US;" in freertos
    assert "moves[1].pulse_us = DRV_COAX_CTRL_SERVO_CENTER_US;" in freertos


def test_imu_spi_timeout_is_short_but_not_overly_aggressive() -> None:
    board = read("BSP/Src/bsp_board.c")

    assert "#define BSP_IMU_SPI_TIMEOUT_MS 5U" in board
    assert "imu_bus.timeout_ms = BSP_IMU_SPI_TIMEOUT_MS;" in board
    assert "imu_bus.timeout_ms = 100U;" not in board
