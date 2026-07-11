from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def consume_tfmini(data: list[int]):
    frame: list[int] = []
    frames = 0
    checksum_errors = 0
    latest = None

    for byte in data:
        if not frame:
            if byte == 0x59:
                frame.append(byte)
            continue
        if len(frame) == 1:
            if byte == 0x59:
                frame.append(byte)
            else:
                frame.clear()
            continue
        frame.append(byte)
        if len(frame) != 9:
            continue
        if (sum(frame[:8]) & 0xFF) == frame[8]:
            frames += 1
            latest = {
                "distance_cm": frame[2] | (frame[3] << 8),
                "strength": frame[4] | (frame[5] << 8),
                "temperature_c": (frame[6] | (frame[7] << 8)) / 8.0 - 256.0,
            }
        else:
            checksum_errors += 1
        frame.clear()
        if byte == 0x59:
            frame.append(byte)

    return frames, checksum_errors, latest


def make_frame(distance_cm: int, strength: int, temperature_raw: int) -> list[int]:
    frame = [
        0x59,
        0x59,
        distance_cm & 0xFF,
        (distance_cm >> 8) & 0xFF,
        strength & 0xFF,
        (strength >> 8) & 0xFF,
        temperature_raw & 0xFF,
        (temperature_raw >> 8) & 0xFF,
    ]
    frame.append(sum(frame) & 0xFF)
    return frame


def test_tfmini_parser_accepts_documented_nine_byte_frame() -> None:
    frames, checksum_errors, latest = consume_tfmini(make_frame(123, 456, 2088))

    assert frames == 1
    assert checksum_errors == 0
    assert latest == {
        "distance_cm": 123,
        "strength": 456,
        "temperature_c": 5.0,
    }


def test_tfmini_parser_resynchronizes_after_noise_and_bad_checksum() -> None:
    bad = make_frame(50, 100, 2048)
    bad[-1] ^= 1
    good = make_frame(75, 200, 2048)

    frames, checksum_errors, latest = consume_tfmini([0, 0x59, 0x12] + bad + good)

    assert frames == 1
    assert checksum_errors == 1
    assert latest["distance_cm"] == 75


def test_uart8_rangefinder_is_wired_through_project_layers() -> None:
    cmake = read("CMakeLists.txt")
    board = read("BSP/Src/bsp_board.c")
    uart = read("App/Src/app_uart.c")
    freertos = read("Core/Src/freertos.c")
    control = read("App/Src/app_control.c")

    assert "Driver/Src/drv_tfmini.c" in cmake
    assert "BSP/Src/bsp_rangefinder.c" in cmake
    assert "App/Src/app_rangefinder.c" in cmake
    assert "rangefinder_bus.huart = &huart8;" in board
    assert "APP_Rangefinder_Init();" in uart
    assert "APP_Rangefinder_Step();" in uart
    assert "BSP_Rangefinder_OnUartRxEvent(huart, Size);" in uart
    assert "BSP_Rangefinder_OnUartError(huart);" in uart
    assert "0x5AU, 0x05U, 0x05U, 0x01U, 0x65U" in read("Driver/Src/drv_tfmini.c")
    assert 'strcmp(tokens[0], "RANGE?") == 0' in control


def test_rangefinder_feeds_flow_scale_and_altitude_control() -> None:
    rangefinder = read("App/Src/app_rangefinder.c")
    flow = read("App/Src/app_optical_flow.c")
    freertos = read("Core/Src/freertos.c")

    assert "APP_OpticalFlow_UpdateHeightFromRange" in rangefinder
    assert "APP_Rangefinder_GetHeightSample" in freertos
    assert "flow_ctx.height_m = height_m;" in flow
    assert "flow_ctx.height_raw_m = raw_height_m;" in flow
    assert "flow_ctx.height_valid = valid;" in flow
    assert "attitude.z_m = -range_height_m;" in freertos
    assert "attitude.vz_m_s = -range_velocity_m_s;" in freertos
    assert "reference.z_m = -stabilizer_rc_throttle_01" in freertos
    assert "STABILIZER_ALT_HOLD_CORRECTION_LIMIT_US 80" in freertos
    assert "altitude_correction_us = 0;" in freertos
    assert "range_height_valid" in freertos


def test_vofa_channel_three_reports_rangefinder_height_without_changing_frame_size() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define VOFA_DATA_SIZE 24U" in freertos
    assert "APP_Rangefinder_GetStatus(&range_status);" in freertos
    assert "vofa_data[3] = (range_status.valid != 0U) ?" in freertos
    assert "range_status.height_m : 0.0f;" in freertos


def test_vofa_compact_frame_keeps_dashboard_velocity_channels() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define VOFA_DATA_SIZE 24U" in freertos
    assert "vofa_data[5] = vofa_debug.vel_est_m_s[0];" in freertos
    assert "vofa_data[6] = vofa_debug.vel_est_m_s[1];" in freertos
