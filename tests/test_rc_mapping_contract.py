from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_freertos_documents_fixed_elrs_channel_map() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "ELRS / CRSF 遥控器通道约定" in freertos
    assert "CH1 → 左右 / roll stick" in freertos
    assert "CH2 → 前后 / pitch stick" in freertos
    assert "CH3 → 左摇杆上下，回中油门" in freertos
    assert "CH4 → 偏航 / yaw stick" in freertos
    assert "CH5 → 二值开关 / arm switch" in freertos
    assert "+100=开锁，-100=关锁" in freertos


def test_controller_uses_named_rc_channels_for_references() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_RC_CH_ROLL          0U" in freertos
    assert "#define STABILIZER_RC_CH_PITCH         1U" in freertos
    assert "#define STABILIZER_RC_CH_THROTTLE_Z    2U" in freertos
    assert "#define STABILIZER_RC_CH_YAW           3U" in freertos
    assert "#define STABILIZER_RC_CH_ARM           4U" in freertos
    assert "#define STABILIZER_RC_ARM_THRESHOLD_US 1500U" in freertos
    assert "#define STABILIZER_XY_REF_RANGE_M      1.20f" in freertos
    assert "reference.x_m = stabilizer_rc_normalized(ch[STABILIZER_RC_CH_PITCH])" in freertos
    assert "reference.y_m = stabilizer_rc_normalized(ch[STABILIZER_RC_CH_ROLL])" in freertos
    assert "reference.z_m = stabilizer_rc_normalized(ch[STABILIZER_RC_CH_THROTTLE_Z])" in freertos
    assert "reference.yaw_rad = stabilizer_rc_normalized(ch[STABILIZER_RC_CH_YAW])" in freertos


def test_controller_mode_keeps_rc_direct_tilt_as_explicit_debug_switch() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_USE_RC_DIRECT_TILT_SERVO 0U" in freertos
    assert "#define STABILIZER_RC_DIRECT_TILT_LIMIT_RAD 0.261799395f" in freertos
    assert "static void stabilizer_map_rc_direct_to_servo" in freertos
    assert "#if (STABILIZER_USE_RC_DIRECT_TILT_SERVO != 0U)\nstatic void stabilizer_map_rc_direct_to_servo" in freertos
    assert "stabilizer_rc_normalized(ch[STABILIZER_RC_CH_PITCH]) *" in freertos
    assert "stabilizer_rc_normalized(ch[STABILIZER_RC_CH_ROLL]) *" in freertos
    assert "moves[0].pulse_us = DRV_COAX_CTRL_TiltRadToServoPulse(alpha_rad);" in freertos
    assert "moves[1].pulse_us = DRV_COAX_CTRL_TiltRadToServoPulse(beta_rad);" in freertos
    assert "stabilizer_map_rc_direct_to_servo(ch, moves);" in freertos
    assert "未接位置/速度估计" in freertos


def test_arm_switch_gates_motor_output_but_not_controller_reference() -> None:
    freertos = read("Core/Src/freertos.c")
    header = read("Driver/Inc/drv_coax_ctrl.h")
    source = read("Driver/Src/drv_coax_ctrl.c")

    assert "static uint8_t stabilizer_rc_is_armed(const uint16_t ch[CRSF_CHANNEL_COUNT])" in freertos
    assert "rc_armed = stabilizer_rc_is_armed(ch);" in freertos
    assert "if ((rc_armed != 0U) && (imu_control_valid != 0U))" in freertos
    assert "DRV_COAX_CTRL_OmegaToMotorPulse(ctrl_out.omega_upper)" in freertos
    assert "DRV_COAX_CTRL_OmegaToMotorPulse(ctrl_out.omega_lower)" in freertos
    assert "BSP_PWM_SetEscPulse(1, BSP_PWM_ESC_MIN_US);" in freertos
    assert "BSP_PWM_SetEscPulse(2, BSP_PWM_ESC_MIN_US);" in freertos
    assert "uint16_t DRV_COAX_CTRL_OmegaToMotorPulse(float omega_rad_s);" in header
    assert "DRV_COAX_CTRL_MOTOR_OMEGA_MAX_RAD_S 900.0f" in source
