from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "ros_ws" / "src" / "robotcore_control"
sys.path.insert(0, str(PACKAGE_ROOT))

from robotcore_control.control_math import (  # noqa: E402
    ConditionalPid,
    PidGains,
    altitude_collective_pwm_commands,
    altitude_level_pwm_commands,
    altitude_station_pwm_commands,
    altitude_velocity_setpoint,
    first_order_low_pass,
    level_attitude_pd_efforts,
    quaternion_apply,
    quaternion_error_body,
    quaternion_slerp,
    reject_vector_outlier,
    rpy_to_quaternion,
    timestamped_rate_prediction,
)


def test_altitude_setpoint_uses_only_target_and_actual_map_height():
    assert np.isclose(
        altitude_velocity_setpoint(0.8, 0.6, 0.0, 0.5), 0.1
    )
    assert np.isclose(
        altitude_velocity_setpoint(0.2, 0.8, 0.0, 1.0), -0.6
    )


def test_positive_flu_altitude_effort_uses_verified_negative_hardware_pwm():
    commands = altitude_collective_pwm_commands(0.31, 0.4, -1.0)

    assert np.allclose(commands[:4], [-0.31] * 4)
    assert np.allclose(commands[4:], np.zeros(4))
    assert np.allclose(
        altitude_collective_pwm_commands(0.8, 0.4, -1.0)[:4], [-0.4] * 4
    )


def test_fast_station_vertical_mixer_combines_height_roll_and_pitch():
    commands = altitude_level_pwm_commands(
        0.20, 0.04, -0.03, 0.40, -1.0
    )

    assert np.allclose(commands[:4], [-0.19, -0.13, -0.27, -0.21])
    assert np.allclose(commands[4:], np.zeros(4))


def test_fast_station_vertical_mixer_preserves_level_control_at_individual_200us_limit():
    commands = altitude_level_pwm_commands(
        0.40, 0.08, 0.04, 0.40, -1.0
    )

    # 0.40 normalized is one channel's 200 us offset on the 500 us hardware
    # span. Attitude correction keeps its differential; collective height
    # backs off enough that no individual T1--T4 channel exceeds that bound.
    assert np.max(np.abs(commands)) <= 0.40 + 1e-12
    assert np.isclose(commands[0] - commands[3], 0.24)
    assert np.isclose(np.mean(commands[:4]), -0.28)


def test_level_pd_keeps_angle_stiffness_independent_from_delayed_rate_gain():
    efforts, raw, saturated = level_attitude_pd_efforts(
        orientation_error=[0.10, -0.04],
        target_angular_rate=[0.0, 0.0],
        measured_angular_rate=[0.20, -0.10],
        angle_kp=[0.45, 0.45],
        rate_kp=[0.30, 0.55],
        combined_limit=0.10,
    )

    assert np.allclose(raw, [-0.015, 0.037])
    assert np.allclose(efforts, raw)
    assert saturated is False


def test_level_pd_preserves_axis_ratio_when_combined_output_is_limited():
    efforts, raw, saturated = level_attitude_pd_efforts(
        orientation_error=[0.0, 0.0],
        target_angular_rate=[0.0, 0.0],
        measured_angular_rate=[-0.50, 0.20],
        angle_kp=[0.45, 0.45],
        rate_kp=[0.30, 0.55],
        combined_limit=0.10,
    )

    assert np.allclose(raw, [0.15, -0.11])
    assert np.allclose(efforts, raw * (0.10 / 0.26))
    assert saturated is True


def test_station_heading_uses_measured_t5_to_t8_yaw_directions_and_balance():
    counter_clockwise = altitude_station_pwm_commands(
        0.25, 0.0, 0.0, 0.08, 0.4, -1.0
    )
    clockwise = altitude_station_pwm_commands(
        0.25, 0.0, 0.0, -0.08, 0.4, -1.0
    )

    assert np.allclose(counter_clockwise[:4], [-0.25] * 4)
    assert np.allclose(counter_clockwise[4:], [0.08, -0.07984, -0.0284, 0.05776])
    assert np.allclose(clockwise[4:], [-0.07984, 0.08, 0.05776, -0.0284])


def test_station_left_stick_surge_uses_normal_upper_thruster_pattern():
    forward = altitude_station_pwm_commands(0.0, 0.12, 0.0, 0.0, 0.4, -1.0)

    assert np.allclose(forward[4:], [-0.12, -0.12, 0.12, 0.12])


def test_fast_station_full_surge_reaches_individual_200us_channel_limit():
    forward = altitude_station_pwm_commands(0.0, 0.40, 0.0, 0.0, 0.40, -1.0)

    assert np.allclose(forward[4:], [-0.40, -0.40, 0.40, 0.40])
    assert np.max(np.abs(forward)) == 0.40


def test_station_sway_uses_direction_specific_measured_curve_balance():
    positive = altitude_station_pwm_commands(0.0, 0.0, 0.08, 0.0, 0.4, -1.0)
    negative = altitude_station_pwm_commands(0.0, 0.0, -0.08, 0.0, 0.4, -1.0)

    assert np.allclose(positive[4:], [-0.08, 0.0, -0.07144, 0.04864])
    assert np.allclose(negative[4:], [0.0, -0.08, 0.04864, -0.07144])


def test_station_combined_axes_preserve_ratios_at_live_pwm_limit():
    reference = altitude_station_pwm_commands(
        0.0, 0.08, 0.08, 0.08, 1.0, -1.0
    )[4:]
    limited = altitude_station_pwm_commands(
        0.0, 0.08, 0.08, 0.08, 0.10, -1.0
    )[4:]

    assert np.max(np.abs(limited)) <= 0.10 + 1e-12
    assert np.allclose(limited, reference * (0.10 / np.max(np.abs(reference))))


def test_altitude_pid_anti_windup_uses_live_pwm_limit():
    pid = ConditionalPid(
        PidGains(2.0, 0.6, 0.0, 0.4, 1.0)
    )

    output = pid.step(
        setpoint=0.2, measurement=0.0, dt=0.05, output_limit=0.2
    )

    assert np.isclose(output, 0.2)
    assert pid.saturated is True
    assert np.isclose(pid.integral, 0.0)


def test_altitude_velocity_filter_smooths_measurement_not_pwm_command():
    assert np.isclose(first_order_low_pass(None, 0.1, 0.03, 0.2), 0.1)
    filtered = first_order_low_pass(0.0, 0.1, 0.1, 0.2)
    assert 0.0 < filtered < 0.1


def test_rate_outlier_rejection_has_no_delay_for_normal_motion():
    history = [[0.00, 0.0, 0.0], [0.02, 0.0, 0.0]]

    assert np.allclose(
        reject_vector_outlier(history, [0.04, 0.0, 0.0], 0.20),
        [0.04, 0.0, 0.0],
    )
    assert np.allclose(
        reject_vector_outlier(history, [1.00, 0.0, 0.0], 0.20),
        [0.02, 0.0, 0.0],
    )


def test_timestamped_rate_prediction_follows_linear_motion_to_apply_time():
    times_ns = [0, 10_000_000, 20_000_000, 30_000_000, 40_000_000]
    rates = [[0.01 * index, 0.0, 0.0] for index in range(5)]

    predicted = timestamped_rate_prediction(
        times_ns, rates, 0.04, 4.0, 0.12
    )

    assert np.allclose(predicted, [0.08, 0.0, 0.0], atol=1e-12)


def test_timestamped_rate_prediction_bounds_noisy_extrapolation():
    predicted = timestamped_rate_prediction(
        [0, 10_000_000, 20_000_000],
        [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, -1.0, 0.0]],
        0.04,
        4.0,
        0.12,
    )

    assert np.all(np.abs(predicted - [1.0, -1.0, 0.0]) <= 0.12 + 1e-12)


def test_quaternion_error_is_sign_invariant_and_uses_shortest_path():
    current = rpy_to_quaternion(0.0, 0.0, np.deg2rad(179.0))
    target = rpy_to_quaternion(0.0, 0.0, np.deg2rad(-179.0))
    error = quaternion_error_body(current, target)
    negated = quaternion_error_body(-current, -target)
    assert np.allclose(error, negated, atol=1e-10)
    assert np.isclose(abs(error[2]), np.deg2rad(2.0), atol=1e-6)


def test_quaternion_rotation_uses_world_from_body_convention():
    yaw_90 = rpy_to_quaternion(0.0, 0.0, np.pi / 2.0)
    rotated = quaternion_apply(yaw_90, [1.0, 0.0, 0.0])
    assert np.allclose(rotated, [0.0, 1.0, 0.0], atol=1e-7)


def test_quaternion_slerp_is_sign_invariant_and_follows_shortest_arc():
    start = rpy_to_quaternion(0.0, 0.0, np.deg2rad(170.0))
    target = rpy_to_quaternion(0.0, 0.0, np.deg2rad(-170.0))

    midpoint = quaternion_slerp(start, target, 0.5)
    negated_midpoint = quaternion_slerp(-start, -target, 0.5)

    assert np.isclose(abs(midpoint[3]), 1.0, atol=1e-6)
    assert np.isclose(abs(np.dot(midpoint, negated_midpoint)), 1.0, atol=1e-10)


def test_conditional_pid_does_not_wind_up_while_saturated():
    pid = ConditionalPid(
        PidGains(
            kp=2.0,
            ki=1.0,
            kd=0.0,
            integral_limit=100.0,
            output_limit=1.0,
        )
    )
    outputs = [
        pid.step(setpoint=10.0, measurement=0.0, dt=0.1)
        for _ in range(100)
    ]
    assert outputs == [1.0] * 100
    assert pid.integral == 0.0

    # Reversing the error must immediately pull the command off the positive
    # stop instead of first unwinding a hidden integral.
    output = pid.step(setpoint=-0.1, measurement=0.0, dt=0.1)
    assert output < 0.0


def test_pid_reset_clears_derivative_and_integral_state():
    pid = ConditionalPid(PidGains(1.0, 1.0, 1.0, 5.0, 10.0))
    pid.step(setpoint=1.0, measurement=0.2, dt=0.1)
    pid.step(setpoint=1.0, measurement=0.4, dt=0.1)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.previous_measurement is None
    assert pid.filtered_derivative == 0.0


def test_pd_mode_omits_and_clears_integral_state():
    pid = ConditionalPid(PidGains(1.0, 4.0, 0.0, 5.0, 100.0))
    pid.step(setpoint=1.0, measurement=0.0, dt=0.5)
    assert pid.integral > 0.0

    output = pid.step(
        setpoint=1.0,
        measurement=0.0,
        dt=0.5,
        integral_enabled=False,
    )

    assert np.isclose(output, 1.0)
    assert np.isclose(pid.integral, 0.0)
