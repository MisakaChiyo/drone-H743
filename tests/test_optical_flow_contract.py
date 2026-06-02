from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def make_frame(fx: int, fy: int, dt_us: int, ground: int = 999, valid: int = 0xF5, version: int = 1) -> list[int]:
    payload = []
    for value in (fx, fy, dt_us, ground):
        value &= 0xFFFF
        payload.extend([value & 0xFF, (value >> 8) & 0xFF])
    payload.extend([valid, version])
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return [0xFE, 0x0A, *payload, checksum, 0x55]


def consume_python(frame_bytes: list[int]):
    offset = 0
    buf = [0] * 14
    frames = 0
    checksum_errors = 0
    frame_errors = 0
    latest = None

    def s16(lo: int, hi: int) -> int:
        value = lo | (hi << 8)
        return value - 0x10000 if value & 0x8000 else value

    for byte in frame_bytes:
        if offset == 0:
            if byte != 0xFE:
                continue
            buf[offset] = byte
            offset += 1
            continue
        if offset == 1:
            if byte != 0x0A:
                frame_errors += 1
                offset = 0
                if byte == 0xFE:
                    buf[offset] = byte
                    offset += 1
                continue
            buf[offset] = byte
            offset += 1
            continue
        buf[offset] = byte
        offset += 1
        if offset < 14:
            continue
        offset = 0
        if buf[13] != 0x55:
            frame_errors += 1
            if byte == 0xFE:
                buf[offset] = byte
                offset += 1
            continue
        checksum = 0
        for payload_byte in buf[2:12]:
            checksum ^= payload_byte
        if checksum != buf[12]:
            checksum_errors += 1
            continue
        frames += 1
        latest = {
            "fx": s16(buf[2], buf[3]),
            "fy": s16(buf[4], buf[5]),
            "dt_us": buf[6] | (buf[7] << 8),
            "ground": buf[8] | (buf[9] << 8),
            "valid": buf[10],
            "version": buf[11],
        }
    return frames, checksum_errors, frame_errors, latest


def test_lc302_lc307_frame_parser_accepts_valid_frame() -> None:
    frames, cksum, frame_errors, latest = consume_python(make_frame(123, -45, 20000))

    assert frames == 1
    assert cksum == 0
    assert frame_errors == 0
    assert latest == {
        "fx": 123,
        "fy": -45,
        "dt_us": 20000,
        "ground": 999,
        "valid": 0xF5,
        "version": 1,
    }


def test_lc302_lc307_frame_parser_rejects_bad_checksum_and_tail() -> None:
    bad_checksum = make_frame(1, 2, 20000)
    bad_checksum[12] ^= 0x01
    bad_tail = make_frame(3, 4, 20000)
    bad_tail[13] = 0xAA

    frames, cksum, frame_errors, latest = consume_python(bad_checksum + bad_tail)

    assert frames == 0
    assert cksum == 1
    assert frame_errors == 1
    assert latest is None


def test_lc302_lc307_parser_resynchronizes_after_noise() -> None:
    stream = [0x00, 0xFE, 0x09, 0x11, 0xFE] + make_frame(-10, 20, 18000)

    frames, cksum, frame_errors, latest = consume_python(stream)

    assert frames == 1
    assert cksum == 0
    assert frame_errors == 2
    assert latest["fx"] == -10
    assert latest["fy"] == 20


def test_optical_flow_sources_are_wired_into_firmware() -> None:
    cmake = read("CMakeLists.txt")
    uart = read("App/Src/app_uart.c")
    freertos = read("Core/Src/freertos.c")
    usart = read("Core/Src/usart.c")
    ioc = read("drone-H743.ioc")
    control = read("App/Src/app_control.c")

    assert "Driver/Src/drv_optical_flow.c" in cmake
    assert "BSP/Src/bsp_optical_flow.c" in cmake
    assert "App/Src/app_optical_flow.c" in cmake
    assert "BSP_OPTICAL_FLOW_OnUartRxCplt(huart);" in uart
    assert "BSP_OPTICAL_FLOW_OnUartError(huart);" in uart
    assert "BSP_GPS_OnUartRxCplt" not in uart
    assert "BSP_GPS_OnUartError" not in uart
    assert "huart2.Init.BaudRate = 19200;" in usart
    assert "USART2.BaudRate=19200" in ioc
    assert "FLOW?" in control
    assert "APP_OpticalFlow_Report();" in control
    assert "FLOW PINGAB" in control
    assert "BSP_OPTICAL_FLOW_TransmitRaw" in control
    assert "APP_Task_OpticalFlow_Init();" in freertos
    assert "APP_Task_OpticalFlow_Step();" in freertos


def test_velocity_source_prefers_flow_and_falls_back_to_imu() -> None:
    freertos = read("Core/Src/freertos.c")
    app_flow = read("App/Src/app_optical_flow.c")
    app_flow_header = read("App/Inc/app_optical_flow.h")

    assert "velocity_imu_x_m_s = nav_state.vel_m_s[0];" in freertos
    assert "APP_OpticalFlow_GetVelocitySample(&flow_vx_m_s," in freertos
    assert "stabilizer_velocity_estimator_step(&vel_estimator," in freertos
    assert "state->vel_m_s[0] += acc_x_m_s2 * dt_sec;" in freertos
    assert "state->flow_lpf_m_s[0] += STABILIZER_FLOW_VEL_LPF_ALPHA *" in freertos
    assert "STABILIZER_FLOW_CORRECTION_GAIN *" in freertos
    assert "flow_sample_ms != state->last_flow_sample_ms" in freertos
    assert "velocity_state_x_m_s = vel_estimator.vel_m_s[0];" in freertos
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_IMU);" in freertos
    assert "APP_OpticalFlow_GetVelocitySample" in app_flow_header
    assert "uint32_t *sample_ms" in app_flow_header
    assert "return APP_OpticalFlow_GetVelocitySample(vx_m_s, vy_m_s, &sample_ms);" in app_flow
    assert "*sample_ms = flow_ctx.bsp_status.latest.received_ms;" in app_flow
    assert "flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_FLOW;" in app_flow
    assert "flow_ctx.velocity_source = APP_OPTICAL_FLOW_VEL_SOURCE_IMU;" in app_flow
    assert "frame->valid != DRV_OPTICAL_FLOW_VALID" in app_flow
    assert "age_ms > APP_FLOW_TIMEOUT_MS" in app_flow
    assert "flow_ctx.height_valid == 0U" in app_flow


def test_optical_flow_mount_rotation_and_fixed_height_are_applied_in_app_layer() -> None:
    app_flow = read("App/Src/app_optical_flow.c")
    header = read("App/Inc/app_optical_flow.h")

    assert "APP_FLOW_MOUNT_COS_45" in app_flow
    assert "APP_FLOW_MOUNT_SIN_45" in app_flow
    assert "sensor_vx_m_s" in app_flow
    assert "sensor_vy_m_s" in app_flow
    assert "body_vx_m_s = APP_FLOW_MOUNT_COS_45 * sensor_vx_m_s -" in app_flow
    assert "body_vy_m_s = APP_FLOW_MOUNT_SIN_45 * sensor_vx_m_s +" in app_flow
    assert "flow_ctx.vx_m_s = -body_vx_m_s;" in app_flow
    assert "flow_ctx.vy_m_s = -body_vy_m_s;" in app_flow
    assert "APP_FLOW_FIXED_HEIGHT_M      0.06f" in app_flow
    assert "APP_FLOW_HEIGHT_LPF_ALPHA" in app_flow
    assert "flow_ctx.height_m = APP_FLOW_FIXED_HEIGHT_M;" in app_flow
    assert "flow_ctx.height_raw_m = APP_FLOW_FIXED_HEIGHT_M;" in app_flow
    assert "(void)pressure_pa;" in app_flow
    assert "(void)fresh;" in app_flow
    assert "powf(" not in app_flow
    assert "height_raw_m" in header
    assert "height_filter_alpha" in header
    assert "height_raw_mm" in app_flow


def test_flow_report_includes_recent_raw_frame_statistics() -> None:
    driver_header = read("Driver/Inc/drv_optical_flow.h")
    driver = read("Driver/Src/drv_optical_flow.c")
    bsp_header = read("BSP/Inc/bsp_optical_flow.h")
    bsp = read("BSP/Src/bsp_optical_flow.c")
    app_header = read("App/Inc/app_optical_flow.h")
    app = read("App/Src/app_optical_flow.c")

    assert "#define DRV_OPTICAL_FLOW_RAW_WINDOW 32U" in driver_header
    assert "DRV_OPTICAL_FLOW_RawStats" in driver_header
    assert "raw_window[DRV_OPTICAL_FLOW_RAW_WINDOW]" in driver_header
    assert "flow_update_raw_stats(dev, &dev->latest);" in driver
    assert "flow_x_peak_to_peak" in driver
    assert "integration_timespan_peak_to_peak_us" in driver
    assert "typedef DRV_OPTICAL_FLOW_RawStats BSP_OPTICAL_FLOW_RawStats;" in bsp_header
    assert "status->raw_stats = flow_dev.raw_stats;" in bsp
    assert "raw_count" in app_header
    assert "flow_x_mean" in app_header
    assert "ground_distance_peak_to_peak" in app_header
    assert "FLOW raw n=%u fx_avg=%d fy_avg=%d dt_avg=%u ground_avg=%u fx_pp=%d fy_pp=%d dt_pp=%u ground_pp=%u" in app


def test_lc307_initialization_protocol_is_explicit_and_does_not_reuse_lc306_table() -> None:
    driver = read("Driver/Src/drv_optical_flow.c")
    app = read("App/Src/app_optical_flow.c")
    source = app + driver

    assert "Config_Init_Uart" not in source
    assert "Sensor_cfg" not in source
    assert "LC307_CMD_AA" in driver
    assert "LC307_CMD_AB" in driver
    assert "LC307_CMD_BB" in driver
    assert "LC307_CMD_DD" in driver
    assert "LC307_CONFIG_DELAY_MS      100U" in driver
    assert "LC307_CONFIG_AB_RETRIES" in driver
    assert "lc307_config_table" in driver
    assert "sizeof(lc307_config_table)" in driver
    assert "{0x12U, 0x80U}" in driver
    assert "DRV_OPTICAL_FLOW_CONFIG_MISSING_TABLE" in driver
    assert "cfg_missing" in app
    assert "missing_tbl" in app
    assert "ab_rx=%02X,%02X,%02X" in app


def test_optical_flow_bus_provides_lc307_config_timing_hooks() -> None:
    header = read("Driver/Inc/drv_optical_flow.h")
    board = read("BSP/Src/bsp_board.c")

    assert "uint32_t timeout_ms;" in header
    assert "void (*delay_ms)(uint32_t ms);" in header
    assert "optical_flow_bus.timeout_ms = 100U;" in board
    assert "optical_flow_bus.delay_ms = BSP_DelayMs;" in board
