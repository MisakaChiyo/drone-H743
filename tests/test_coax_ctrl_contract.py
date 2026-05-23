from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_servo_beta_direction_is_inverted_in_hardware_wrapper() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")
    generated = read("Driver/Generated/coax_ctrl/coax_tiltrotor_controller_codegen.c")

    assert "DRV_COAX_CTRL_SERVO_ALPHA_SIGN    (1.0f)" in source
    assert "DRV_COAX_CTRL_SERVO_BETA_SIGN    (-1.0f)" in source
    assert "cmd[2] * DRV_COAX_CTRL_SERVO_ALPHA_SIGN" in source
    assert "cmd[3] * DRV_COAX_CTRL_SERVO_BETA_SIGN" in source
    assert "DRV_COAX_CTRL_SERVO_ALPHA_SIGN" not in generated
    assert "DRV_COAX_CTRL_SERVO_BETA_SIGN" not in generated


def test_tilt_limit_is_fifteen_degrees_in_wrapper_and_generated_controller() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")
    generated = read("Driver/Generated/coax_ctrl/coax_tiltrotor_controller_codegen.c")

    assert "DRV_COAX_CTRL_TILT_LIMIT_RAD 0.261799395f" in source
    assert "params->tilt_limit_rad" in generated


def test_servo_alpha_and_beta_have_separate_mechanical_centers() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    source = read("Driver/Src/drv_coax_ctrl.c")
    freertos = read("Core/Src/freertos.c")

    assert "#define DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US 1412U" in header
    assert "#define DRV_COAX_CTRL_SERVO_BETA_CENTER_US  1851U" in header
    assert "DRV_COAX_CTRL_AlphaTiltRadToServoPulse" in header
    assert "DRV_COAX_CTRL_BetaTiltRadToServoPulse" in header
    assert "coax_ctrl_tilt_rad_to_servo_pulse(float tilt_rad," in source
    assert "DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US" in source
    assert "DRV_COAX_CTRL_SERVO_BETA_CENTER_US" in source
    assert "DRV_COAX_CTRL_AlphaTiltRadToServoPulse(alpha_rad)" in freertos
    assert "DRV_COAX_CTRL_BetaTiltRadToServoPulse(beta_rad)" in freertos
    assert "moves[0].pulse_us = DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US;" in freertos
    assert "moves[1].pulse_us = DRV_COAX_CTRL_SERVO_BETA_CENTER_US;" in freertos


def test_mbd_controller_gains_are_runtime_coax_params() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    wrapper = read("Driver/Src/drv_coax_ctrl.c")
    generated = read("Driver/Generated/coax_ctrl/coax_tiltrotor_controller_codegen.c")
    app_control = read("App/Src/app_control.c")

    assert "DRV_COAX_CTRL_Params" in header
    assert "DRV_COAX_CTRL_SetParam" in header
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(roll_angle_kp)' in wrapper
    assert '"coax." #field' in wrapper
    assert "params->roll_angle_kp" in generated
    assert "params->pitch_rate_kd" in generated
    assert "params->yaw_angle_kp" in generated
    assert "params->yaw_rate_kd" in generated
    assert "DRV_COAX_CTRL_SetParam(name, value)" in app_control
    assert "PARAM name=%s value=%s" in app_control
