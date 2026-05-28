from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_servo_beta_direction_is_inverted_in_hardware_wrapper() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")
    generated = read("Driver/Generated/coax_ctrl/coax_tiltrotor_controller_codegen.c")

    assert "DRV_COAX_CTRL_SERVO_ALPHA_SIGN    (1.0f)" in source
    assert "DRV_COAX_CTRL_SERVO_BETA_SIGN    (-1.0f)" in source
    assert "output->alpha_rad * DRV_COAX_CTRL_SERVO_ALPHA_SIGN" in source
    assert "output->beta_rad * DRV_COAX_CTRL_SERVO_BETA_SIGN" in source
    assert "DRV_COAX_CTRL_SERVO_ALPHA_SIGN" not in generated
    assert "DRV_COAX_CTRL_SERVO_BETA_SIGN" not in generated


def test_tilt_limit_is_ten_degrees_in_wrapper_and_generated_controller() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")
    generated = read("Driver/Generated/coax_ctrl/coax_tiltrotor_controller_codegen.c")

    assert "DRV_COAX_CTRL_TILT_LIMIT_RAD 0.174533f" in source
    assert "params->tilt_limit_rad" in generated


def test_bus_servos_are_270_degree_centered_and_limited_to_90_degrees() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    source = read("Driver/Src/drv_coax_ctrl.c")
    freertos = read("Core/Src/freertos.c")

    assert "#define DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US 1441U" in header
    assert "#define DRV_COAX_CTRL_SERVO_BETA_CENTER_US  1877U" in header
    assert "#define DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US  500U" in header
    assert "#define DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US 2500U" in header
    assert "#define DRV_COAX_CTRL_SERVO_MIN_US           833U" in header
    assert "#define DRV_COAX_CTRL_SERVO_MAX_US          2167U" in header
    assert "#define DRV_COAX_CTRL_SERVO_TRAVEL_DEG       270.0f" in header
    assert "#define DRV_COAX_CTRL_SERVO_LIMIT_DEG         90.0f" in header
    assert "DRV_COAX_CTRL_AlphaTiltRadToServoPulse" in header
    assert "DRV_COAX_CTRL_BetaTiltRadToServoPulse" in header
    assert "coax_ctrl_tilt_rad_to_servo_pulse(float tilt_rad," in source
    assert "DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US" in source
    assert "DRV_COAX_CTRL_SERVO_LIMIT_RAD" in source
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
    assert 'DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_x_kp)' in wrapper
    assert "params->vel_loop_enable = 1.0f;" in wrapper
    assert "params->vel_loop_output_limit_m_s2 = 1.2f;" in wrapper
    assert '"coax." #field' in wrapper
    assert "params->roll_angle_kp" in generated
    assert "params->pitch_rate_kd" in generated
    assert "params->yaw_angle_kp" in generated
    assert "params->yaw_rate_kd" in generated
    assert "DRV_COAX_CTRL_SetParam(name, value)" in app_control
    assert "PARAM name=%s value=%s" in app_control


def test_controller_wrapper_exposes_velocity_first_vector_control_inputs() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    wrapper = read("Driver/Src/drv_coax_ctrl.c")
    freertos = read("Core/Src/freertos.c")

    assert "float vx_m_s;" in header
    assert "float ax_m_s2;" in header
    assert "x_rb[3] = attitude->vx_m_s;" in wrapper
    assert "x_rb[4] = attitude->vy_m_s;" in wrapper
    assert "reference->ax_m_s2" in wrapper
    assert "reference->vx_m_s" in wrapper
    assert "STABILIZER_XY_VEL_REF_MAX_M_S" in freertos
    assert "STABILIZER_XY_ACCEL_LIMIT_M_S2" in freertos
    assert "reference.vx_m_s = stabilizer_rc_normalized(ch[STABILIZER_RC_CH_PITCH])" in freertos
    assert "reference.ax_m_s2 =" in freertos
    assert "float prev_meas_m_s;" in freertos
    assert "d = -kd * (meas_m_s - state->prev_meas_m_s) / dt_sec;" in freertos
    assert "stabilizer_velocity_pid_step(&vel_pid_y,\n                                             -vel_err_y_m_s," in freertos
    assert "position_ref_x_m = 0.0f;" in freertos
    assert "reference.x_m = attitude.x_m;" in freertos
    assert "attitude.vx_m_s = velocity_state_x_m_s;" in freertos
    assert "velocity_ref_x_m_s" not in freertos


def test_roll_pitch_angle_gains_default_to_zero_as_constraints_not_primary_loop() -> None:
    wrapper = read("Driver/Src/drv_coax_ctrl.c")

    assert "params->roll_angle_kp = 0.0f;" in wrapper
    assert "params->pitch_angle_kp = 0.0f;" in wrapper
    assert "params->roll_rate_kd = -0.12f;" in wrapper
    assert "params->pitch_rate_kd = -0.12f;" in wrapper


def test_roll_pitch_tilt_output_is_acceleration_vector_plus_rate_damping_only() -> None:
    wrapper = read("Driver/Src/drv_coax_ctrl.c")
    helper = wrapper.split("static void coax_ctrl_compute_pure_damping_tilt", 1)[1]
    helper = helper.split("static float *coax_ctrl_param_ptr", 1)[0]

    assert "coax_ctrl_compute_pure_damping_tilt(attitude, reference," in wrapper
    assert "acc_x_m_s2 = reference->ax_m_s2 -" in helper
    assert "acc_y_m_s2 = reference->ay_m_s2 -" in helper
    assert "reference->vx_m_s - attitude->vx_m_s" in helper
    assert "reference->vy_m_s - attitude->vy_m_s" in helper
    assert "atan2f(acc_x_m_s2, vertical_acc_m_s2)" in helper
    assert "(coax_ctrl_params.pitch_rate_kd * attitude->gyro_y_rad_s)" in helper
    assert "(coax_ctrl_params.roll_rate_kd * attitude->gyro_x_rad_s)" in helper
    assert "pitch_rad" not in helper
    assert "roll_rad" not in helper
    assert "pitch_angle_kp" not in helper
    assert "roll_angle_kp" not in helper
    assert "output->alpha_rad = alpha_rad;" in wrapper
    assert "output->beta_rad = beta_rad;" in wrapper


def test_vofa_exports_acceleration_controller_pid_tail() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define VOFA_DATA_SIZE 63U" in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.roll_rate_kd", &vofa_data[22]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.pitch_rate_kd", &vofa_data[23]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.yaw_angle_kp", &vofa_data[24]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.yaw_rate_kd", &vofa_data[25]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_x_kd", &vofa_data[26]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_y_kd", &vofa_data[27]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.vel_z_kd", &vofa_data[28]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.accel_xy_limit_m_s2", &vofa_data[29]);' in freertos
    assert '(void)DRV_COAX_CTRL_GetParam("coax.accel_z_limit_m_s2", &vofa_data[30]);' in freertos
    assert "vofa_data[31] = vofa_debug.acc_nav_m_s2[0];" in freertos
    assert "vofa_data[62] = vofa_debug.vel_loop_active;" in freertos
    assert "coax.pos_x_kp" not in freertos
    assert "coax.mass_kg" not in freertos


def test_control_protocol_accepts_colon_param_updates_and_reports_back() -> None:
    app_control = read("App/Src/app_control.c")

    assert "app_control_handle_param_value_line(line)" in app_control
    assert "app_control_after_param_separator" in app_control
    assert "0xEFU" in app_control
    assert "0xBCU" in app_control
    assert "0x9AU" in app_control
    assert "app_control_report_coax_param_by_name(name);" in app_control
    assert "app_control_report_pid_legacy();" in app_control
    assert "app_control_report_coax_param_by_name(map[map_index].param_name);" in app_control
    assert '"roll_rate_kd",   "coax.roll_rate_kd"' in app_control
    assert '"yaw_angle_kp",   "coax.yaw_angle_kp"' in app_control
    assert '"vel_x_kd",       "coax.vel_x_kd"' in app_control
    assert '"vel_loop_x_kp",  "coax.vel_loop_x_kp"' in app_control
    assert '"accel_xy",       "coax.accel_xy_limit_m_s2"' in app_control
    assert '"pos_x_kp",      "coax.pos_x_kp"' not in app_control
