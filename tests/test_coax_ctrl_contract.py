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
    assert "0.261799395F" in generated
