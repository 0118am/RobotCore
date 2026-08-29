"""Pure control mathematics shared by the direct hold controllers.

The module deliberately has no ROS imports so quaternion and anti-windup
behaviour can be tested on development machines without a sourced ROS install.
Quaternions use the ``(w, x, y, z)`` ordering used by the policy code.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


def vec(values: Iterable[float], size: int) -> np.ndarray:
    result = np.asarray(list(values), dtype=np.float64).reshape(-1)
    if result.size != size or not np.all(np.isfinite(result)):
        raise ValueError(f"expected {size} finite values")
    return result


def altitude_velocity_setpoint(
    target_height: float,
    actual_height: float,
    target_vertical_velocity: float,
    position_kp: float,
) -> float:
    """Convert map-frame height error into an unrestricted speed target."""

    values = np.asarray(
        [
            target_height,
            actual_height,
            target_vertical_velocity,
            position_kp,
        ],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("altitude-loop inputs must be finite")
    return float(target_vertical_velocity) + float(position_kp) * (
        float(target_height) - float(actual_height)
    )


def station_velocity_setpoints(
    target_velocity_body: Iterable[float],
    position_error_body: Iterable[float],
    position_kp: Iterable[float],
    surge_limit: float,
    sway_limit: float,
    *,
    center_approach: bool,
) -> np.ndarray:
    """Return planar velocity targets, bypassing caps during center approach."""

    target = vec(target_velocity_body, 2)
    error = vec(position_error_body, 2)
    gains = vec(position_kp, 2)
    limits = vec([surge_limit, sway_limit], 2)
    if np.any(gains < 0.0) or np.any(limits < 0.0):
        raise ValueError("station position gains and speed limits must be non-negative")
    requested = target + gains * error
    if center_approach:
        return requested
    return np.clip(requested, -limits, limits)


def altitude_collective_pwm_commands(
    pid_output: float, command_limit: float, command_sign: float
) -> np.ndarray:
    """Apply one direct PID effort to the four vertical PWM channels."""

    if not all(
        math.isfinite(value) for value in (pid_output, command_limit, command_sign)
    ):
        raise ValueError("altitude PWM inputs must be finite")
    if math.isclose(command_sign, 0.0, abs_tol=1e-12):
        raise ValueError("altitude PWM command sign must be non-zero")
    limit = abs(float(command_limit))
    common = float(
        np.clip(math.copysign(1.0, command_sign) * pid_output, -limit, limit)
    )
    commands = np.zeros(8, dtype=np.float64)
    commands[:4] = common
    return commands


def altitude_level_pwm_commands(
    altitude_pid_output: float,
    roll_pd_output: float,
    pitch_pd_output: float,
    command_limit: float,
    altitude_command_sign: float,
) -> np.ndarray:
    """Mix height with level-attitude correction on vertical T1--T4.

    The installed vertical layout is right-front, right-rear, left-front,
    left-rear. Positive PWM produces down force, so ``(+,+,-,-)`` produces
    positive roll torque and ``(+,-,+,-)`` produces positive pitch torque.
    Attitude differential is preserved first; collective height effort uses
    the remaining symmetric per-thruster PWM headroom.
    """

    if not all(
        math.isfinite(value)
        for value in (
            altitude_pid_output,
            roll_pd_output,
            pitch_pd_output,
            command_limit,
            altitude_command_sign,
        )
    ):
        raise ValueError("altitude-level PWM inputs must be finite")
    if math.isclose(altitude_command_sign, 0.0, abs_tol=1e-12):
        raise ValueError("altitude PWM command sign must be non-zero")

    limit = abs(float(command_limit))
    attitude = (
        float(roll_pd_output) * np.asarray([1.0, 1.0, -1.0, -1.0])
        + float(pitch_pd_output) * np.asarray([1.0, -1.0, 1.0, -1.0])
    )
    attitude_peak = float(np.max(np.abs(attitude)))
    if attitude_peak > limit and attitude_peak > 0.0:
        attitude *= limit / attitude_peak

    requested_collective = math.copysign(
        1.0, altitude_command_sign
    ) * float(altitude_pid_output)
    collective_min = -limit - float(np.min(attitude))
    collective_max = limit - float(np.max(attitude))
    collective = float(
        np.clip(requested_collective, collective_min, collective_max)
    )

    commands = np.zeros(8, dtype=np.float64)
    commands[:4] = np.clip(collective + attitude, -limit, limit)
    return commands


def level_attitude_rate_efforts(
    orientation_error: Iterable[float],
    target_angular_rate: Iterable[float],
    measured_angular_rate: Iterable[float],
    angle_to_rate_kp: Iterable[float],
    rate_kp: Iterable[float],
    angular_rate_limit: Iterable[float],
    combined_limit: float,
    effort_feedforward: Iterable[float] = (0.0, 0.0),
    effort_integral: Iterable[float] = (0.0, 0.0),
) -> tuple[np.ndarray, np.ndarray, np.ndarray, bool]:
    """Run a bounded angle-to-rate cascade for roll and pitch.

    The outer loop converts angle error into a bounded angular-rate setpoint.
    The inner loop converts rate error into actuator-domain effort and adds
    optional model/feedforward and integral terms. The combined L1 limit is
    the peak differential that can appear on any one T1--T4 channel.
    """

    error = vec(orientation_error, 2)
    target_rate = vec(target_angular_rate, 2)
    measured_rate = vec(measured_angular_rate, 2)
    angle_to_rate_gains = vec(angle_to_rate_kp, 2)
    rate_gains = vec(rate_kp, 2)
    rate_limits = vec(angular_rate_limit, 2)
    feedforward = vec(effort_feedforward, 2)
    integral = vec(effort_integral, 2)
    limit = float(combined_limit)
    if (
        np.any(angle_to_rate_gains < 0.0)
        or np.any(rate_gains < 0.0)
        or np.any(rate_limits < 0.0)
        or not math.isfinite(limit)
        or limit < 0.0
    ):
        raise ValueError(
            "level cascade gains, rate limits and effort limit must be "
            "finite and non-negative"
        )

    desired_rate = np.clip(
        target_rate + angle_to_rate_gains * error,
        -rate_limits,
        rate_limits,
    )
    raw = (
        rate_gains * (desired_rate - measured_rate)
        + feedforward
        + integral
    )
    peak = float(np.sum(np.abs(raw)))
    if peak > limit and peak > 0.0:
        return raw * (limit / peak), raw, desired_rate, True
    return raw.copy(), raw, desired_rate, False


def conditional_axis_integral_effort(
    current_effort: float,
    error: float,
    integral_gain: float,
    dt: float,
    effort_limit: float,
    base_efforts: Iterable[float],
    axis: int,
    combined_limit: float,
) -> float:
    """Advance one integral effort without winding up behind an L1 limit.

    ``base_efforts`` contains every non-integral term before the shared
    roll/pitch limiter. Integration is held when it would increase saturation,
    but an update that reduces the combined demand is always allowed so the
    stored effort can unwind after the error reverses.
    """

    base = vec(base_efforts, 2)
    values = (
        current_effort,
        error,
        integral_gain,
        dt,
        effort_limit,
        combined_limit,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("integral effort inputs must be finite")
    if integral_gain < 0.0 or effort_limit < 0.0 or combined_limit < 0.0:
        raise ValueError("integral gain and effort limits must be non-negative")
    if dt <= 0.0:
        raise ValueError("integral timestep must be positive")
    if axis not in (0, 1):
        raise ValueError("integral effort axis must be 0 or 1")

    limit = float(effort_limit)
    current = float(np.clip(current_effort, -limit, limit))
    proposed = float(
        np.clip(
            current + float(integral_gain) * float(error) * float(dt),
            -limit,
            limit,
        )
    )
    current_total = base.copy()
    current_total[axis] += current
    proposed_total = base.copy()
    proposed_total[axis] += proposed
    current_peak = float(np.sum(np.abs(current_total)))
    proposed_peak = float(np.sum(np.abs(proposed_total)))
    if (
        proposed_peak > float(combined_limit) + 1e-12
        and proposed_peak > current_peak + 1e-12
    ):
        return current
    return proposed


def integral_reset_after_error_crossing(
    previous_error_sign: int,
    error: float,
    integral_effort: float,
    hysteresis: float,
) -> tuple[bool, int]:
    """Detect an error zero-crossing that leaves integral effort opposed.

    The sign state changes only outside ``hysteresis`` so sensor noise near
    zero cannot repeatedly reset a useful bias estimate. A crossing resets
    only when the stored integral would drive against the new error.
    """

    values = (error, integral_effort, hysteresis)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("integral crossing inputs must be finite")
    if previous_error_sign not in (-1, 0, 1):
        raise ValueError("previous error sign must be -1, 0, or 1")
    if hysteresis < 0.0:
        raise ValueError("integral crossing hysteresis must be non-negative")
    if abs(error) <= hysteresis:
        return False, previous_error_sign

    current_error_sign = 1 if error > 0.0 else -1
    crossed = previous_error_sign != 0 and current_error_sign != previous_error_sign
    opposing = integral_effort * error < 0.0
    return crossed and opposing, current_error_sign


def surge_pitch_decoupling_effort(
    applied_surge_effort: float,
    forward_gain: float,
    reverse_gain: float,
    output_limit: float,
) -> float:
    """Cancel the measured pitch moment induced by applied surge effort."""

    values = (
        applied_surge_effort,
        forward_gain,
        reverse_gain,
        output_limit,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("surge-pitch decoupling inputs must be finite")
    if forward_gain < 0.0 or reverse_gain < 0.0 or output_limit < 0.0:
        raise ValueError("surge-pitch gains and limit must be non-negative")

    gain = forward_gain if applied_surge_effort >= 0.0 else reverse_gain
    return float(
        np.clip(
            -gain * applied_surge_effort,
            -output_limit,
            output_limit,
        )
    )


def altitude_station_pwm_commands(
    altitude_pid_output: float,
    surge_p_output: float,
    sway_p_output: float,
    yaw_p_output: float,
    command_limit: float,
    altitude_command_sign: float,
    *,
    roll_effort: float = 0.0,
    pitch_effort: float = 0.0,
) -> np.ndarray:
    """Mix height, level and calibrated horizontal station-control outputs."""

    commands = altitude_level_pwm_commands(
        altitude_pid_output,
        roll_effort,
        pitch_effort,
        command_limit,
        altitude_command_sign,
    )
    horizontal, _ = station_horizontal_pwm_mix(
        surge_p_output,
        sway_p_output,
        yaw_p_output,
        command_limit,
    )
    commands[4:] = horizontal
    return commands


def station_horizontal_pwm_mix(
    surge_p_output: float,
    sway_p_output: float,
    yaw_p_output: float,
    command_limit: float,
) -> tuple[np.ndarray, float]:
    """Mix T5--T8 and return the common scale applied at saturation."""

    if not all(
        math.isfinite(value)
        for value in (
            surge_p_output,
            sway_p_output,
            yaw_p_output,
            command_limit,
        )
    ):
        raise ValueError("station horizontal PWM inputs must be finite")
    limit = abs(float(command_limit))
    surge = float(np.clip(surge_p_output, -limit, limit))
    sway = float(np.clip(sway_p_output, -limit, limit))
    yaw = float(np.clip(yaw_p_output, -limit, limit))

    # Forward/back remains the field-verified normal left-stick pattern.
    upper = surge * np.asarray([-1.0, -1.0, 1.0, 1.0])

    # Direction-specific coefficients compensate the measured asymmetric
    # forward/reverse curves. Positive sway is base_link +Y/left. Positive
    # yaw is CCW and deliberately retains T5/T8 positive and T6/T7 negative;
    # its ratios are balanced at the approved 0.20 yaw-command limit.
    if sway >= 0.0:
        upper += sway * np.asarray([-1.0, 0.0, -0.893, 0.608])
    else:
        upper += -sway * np.asarray([0.0, -1.0, 0.608, -0.893])
    if yaw >= 0.0:
        upper += yaw * np.asarray([1.0, -0.998, -0.355, 0.722])
    else:
        upper += -yaw * np.asarray([-0.998, 1.0, 0.722, -0.355])

    # Preserve the requested direction ratios if simultaneous station axes
    # need more authority than the live web PWM limit permits.
    peak = float(np.max(np.abs(upper)))
    scale = 1.0
    if peak > limit and peak > 0.0:
        scale = limit / peak
        upper *= scale
    return upper, scale


def first_order_low_pass(
    previous: float | None,
    sample: float,
    dt: float,
    time_constant_s: float,
) -> float:
    """Filter one finite sample without adding an actuator command ramp."""

    values = (sample, dt, time_constant_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("low-pass inputs must be finite")
    if dt <= 0.0:
        raise ValueError("low-pass dt must be positive")
    if previous is None or time_constant_s <= 0.0:
        return float(sample)
    if not math.isfinite(previous):
        raise ValueError("low-pass previous value must be finite")
    alpha = -math.expm1(-dt / time_constant_s)
    return float(previous + alpha * (sample - previous))


def directional_velocity_feedforward(
    velocity_setpoint: float,
    positive_gain: float,
    negative_gain: float,
    output_limit: float,
) -> float:
    """Map a signed velocity target to bounded actuator-domain feedforward."""

    values = (
        velocity_setpoint,
        positive_gain,
        negative_gain,
        output_limit,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocity feedforward inputs must be finite")
    if positive_gain < 0.0 or negative_gain < 0.0 or output_limit < 0.0:
        raise ValueError("velocity feedforward gains and limit must be non-negative")
    gain = positive_gain if velocity_setpoint >= 0.0 else negative_gain
    return float(
        np.clip(
            gain * velocity_setpoint,
            -output_limit,
            output_limit,
        )
    )


def slew_rate_limit(
    previous: float,
    requested: float,
    dt: float,
    rate_limit_per_s: float,
) -> float:
    """Bound one command's first derivative while preserving its direction."""

    values = (previous, requested, dt, rate_limit_per_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("slew-rate inputs must be finite")
    if dt <= 0.0:
        raise ValueError("slew-rate timestep must be positive")
    if rate_limit_per_s < 0.0:
        raise ValueError("slew-rate limit must be non-negative")
    maximum_delta = rate_limit_per_s * dt
    return float(
        previous
        + np.clip(requested - previous, -maximum_delta, maximum_delta)
    )


def reject_vector_outlier(
    recent_samples: Iterable[Iterable[float]],
    sample: Iterable[float],
    threshold: float,
) -> np.ndarray:
    """Reject an isolated vector spike without delaying ordinary samples.

    The newest sample is compared per axis with the median of itself and the
    two preceding accepted samples. Values inside the threshold pass through
    unchanged; only an implausible single-sample excursion is replaced.
    """

    current = vec(sample, 3)
    limit = float(threshold)
    if not math.isfinite(limit) or limit < 0.0:
        raise ValueError("outlier threshold must be finite and non-negative")
    history = np.asarray(list(recent_samples), dtype=np.float64)
    if history.size == 0 or history.shape[0] < 2 or limit <= 0.0:
        return current.copy()
    if (
        history.ndim != 2
        or history.shape[1] != 3
        or not np.all(np.isfinite(history))
    ):
        raise ValueError("recent vector samples must be finite three-vectors")
    median = np.median(np.vstack((history[-2:], current)), axis=0)
    return np.where(np.abs(current - median) > limit, median, current)


def timestamped_rate_prediction(
    sample_times_ns: Iterable[int],
    rate_samples: Iterable[Iterable[float]],
    prediction_horizon_s: float,
    acceleration_limit_rps2: float,
    correction_limit_rps: float,
) -> np.ndarray:
    """Fit a causal local rate trend and predict a short execution horizon.

    This is an endpoint least-squares polynomial estimate, equivalent to the
    first-order case of a causal Savitzky-Golay filter. Timestamps, rather than
    an assumed sample period, define the fit. Both angular acceleration and
    the total correction from the newest accepted gyro sample are bounded.
    """

    times = np.asarray(list(sample_times_ns), dtype=np.int64).reshape(-1)
    rates = np.asarray(list(rate_samples), dtype=np.float64)
    horizon = float(prediction_horizon_s)
    acceleration_limit = float(acceleration_limit_rps2)
    correction_limit = float(correction_limit_rps)
    if (
        rates.ndim != 2
        or rates.shape[1:] != (3,)
        or rates.shape[0] != times.size
        or times.size == 0
        or not np.all(np.isfinite(rates))
    ):
        raise ValueError(
            "timestamped rates must be a non-empty sequence of three-vectors"
        )
    if np.any(np.diff(times) <= 0):
        raise ValueError("rate sample timestamps must be strictly increasing")
    if not all(
        math.isfinite(value) and value >= 0.0
        for value in (horizon, acceleration_limit, correction_limit)
    ):
        raise ValueError("rate prediction limits must be finite and non-negative")

    latest = rates[-1].copy()
    if (
        times.size < 3
        or horizon <= 0.0
        or acceleration_limit <= 0.0
        or correction_limit <= 0.0
    ):
        return latest

    relative_time = (times - times[-1]).astype(np.float64) * 1e-9
    centered_time = relative_time - float(np.mean(relative_time))
    denominator = float(np.dot(centered_time, centered_time))
    if denominator <= 1e-12:
        return latest
    mean_rate = np.mean(rates, axis=0)
    slope = np.sum(
        centered_time[:, np.newaxis] * (rates - mean_rate), axis=0
    ) / denominator
    slope = np.clip(slope, -acceleration_limit, acceleration_limit)
    fitted_latest = mean_rate - slope * float(np.mean(relative_time))
    predicted = fitted_latest + slope * horizon
    correction = np.clip(
        predicted - latest, -correction_limit, correction_limit
    )
    return latest + correction


def normalize_quaternion(quaternion: Iterable[float]) -> np.ndarray:
    q = vec(quaternion, 4)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-9:
        raise ValueError("quaternion norm is zero")
    return q / norm


def quaternion_slerp(
    current: Iterable[float], target: Iterable[float], weight: float
) -> np.ndarray:
    """Interpolate unit quaternions along the shortest rotation arc."""

    start = normalize_quaternion(current)
    end = normalize_quaternion(target)
    amount = float(np.clip(weight, 0.0, 1.0))
    dot = float(np.dot(start, end))
    if dot < 0.0:
        end = -end
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion(start + amount * (end - start))
    angle = math.acos(dot)
    sine = math.sin(angle)
    return normalize_quaternion(
        math.sin((1.0 - amount) * angle) / sine * start
        + math.sin(amount * angle) / sine * end
    )


def quaternion_conjugate(quaternion: Iterable[float]) -> np.ndarray:
    q = normalize_quaternion(quaternion)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quaternion_multiply(left: Iterable[float], right: Iterable[float]) -> np.ndarray:
    lw, lx, ly, lz = normalize_quaternion(left)
    rw, rx, ry, rz = normalize_quaternion(right)
    return normalize_quaternion(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def quaternion_apply(quaternion: Iterable[float], vector: Iterable[float]) -> np.ndarray:
    q = normalize_quaternion(quaternion)
    value = vec(vector, 3)
    xyz = q[1:4]
    twice_cross = 2.0 * np.cross(xyz, value)
    return value + q[0] * twice_cross + np.cross(xyz, twice_cross)


def quaternion_error_body(
    current_world_from_body: Iterable[float],
    target_world_from_body: Iterable[float],
) -> np.ndarray:
    """Return the shortest target rotation as a body-frame rotation vector."""

    relative = quaternion_multiply(
        quaternion_conjugate(current_world_from_body),
        target_world_from_body,
    )
    if relative[0] < 0.0:
        relative = -relative
    vector_norm = float(np.linalg.norm(relative[1:4]))
    if vector_norm <= 1e-10:
        return 2.0 * relative[1:4]
    angle = 2.0 * math.atan2(vector_norm, max(0.0, float(relative[0])))
    return relative[1:4] * (angle / vector_norm)


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert fixed-axis roll/pitch/yaw radians to a normalized quaternion."""

    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return normalize_quaternion(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quaternion_to_rpy(quaternion: Iterable[float]) -> np.ndarray:
    """Convert a normalized quaternion to fixed-axis roll/pitch/yaw radians."""

    w, x, y, z = normalize_quaternion(quaternion)
    return np.asarray(
        [
            math.atan2(
                2.0 * (w * x + y * z),
                1.0 - 2.0 * (x * x + y * y),
            ),
            math.asin(float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))),
            math.atan2(
                2.0 * (w * z + x * y),
                1.0 - 2.0 * (y * y + z * z),
            ),
        ],
        dtype=np.float64,
    )


def attitude_with_heading(
    tilt_quaternion: Iterable[float], heading_quaternion: Iterable[float]
) -> np.ndarray:
    """Combine IMU roll/pitch with an independent absolute-yaw reference."""

    tilt_rpy = quaternion_to_rpy(tilt_quaternion)
    heading_yaw = quaternion_to_rpy(heading_quaternion)[2]
    return rpy_to_quaternion(
        float(tilt_rpy[0]), float(tilt_rpy[1]), float(heading_yaw)
    )


def quaternion_to_rotation_matrix(quaternion: Iterable[float]) -> np.ndarray:
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * w), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class PidGains:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float
    derivative_cutoff_hz: float = 5.0


class ConditionalPid:
    """PID with derivative-on-measurement and conditional integration."""

    def __init__(self, gains: PidGains):
        self.gains = gains
        self.integral = 0.0
        self.previous_measurement: float | None = None
        self.filtered_derivative = 0.0
        self.saturated = False

    def reset(self):
        self.integral = 0.0
        self.previous_measurement = None
        self.filtered_derivative = 0.0
        self.saturated = False

    def step(
        self,
        *,
        setpoint: float,
        measurement: float,
        dt: float,
        feedforward: float = 0.0,
        output_limit: float | None = None,
        integral_enabled: bool = True,
    ) -> float:
        if not all(math.isfinite(value) for value in (setpoint, measurement, dt, feedforward)):
            raise ValueError("PID inputs must be finite")
        if dt <= 0.0:
            raise ValueError("PID dt must be positive")

        error = float(setpoint - measurement)
        raw_derivative = 0.0
        if self.previous_measurement is not None:
            raw_derivative = -(measurement - self.previous_measurement) / dt
        self.previous_measurement = float(measurement)

        cutoff = max(0.0, float(self.gains.derivative_cutoff_hz))
        alpha = 1.0 if cutoff <= 0.0 else 1.0 - math.exp(-2.0 * math.pi * cutoff * dt)
        self.filtered_derivative += alpha * (raw_derivative - self.filtered_derivative)

        if integral_enabled:
            proposed_integral = float(
                np.clip(
                    self.integral + error * dt,
                    -abs(self.gains.integral_limit),
                    abs(self.gains.integral_limit),
                )
            )
        else:
            # PD operation must not retain integral state accumulated by a
            # previous controller mode.
            self.integral = 0.0
            proposed_integral = 0.0
        unsaturated = (
            feedforward
            + self.gains.kp * error
            + self.gains.ki * proposed_integral
            + self.gains.kd * self.filtered_derivative
        )
        limit = abs(
            float(self.gains.output_limit)
            if output_limit is None
            else float(output_limit)
        )
        if not math.isfinite(limit):
            raise ValueError("PID output limit must be finite")
        saturated = float(np.clip(unsaturated, -limit, limit))

        # Integrate when unsaturated, or when the error would pull an already
        # saturated command back toward the feasible interval.
        can_integrate = (
            math.isclose(unsaturated, saturated, rel_tol=0.0, abs_tol=1e-12)
            or (unsaturated > limit and error < 0.0)
            or (unsaturated < -limit and error > 0.0)
        )
        if integral_enabled and can_integrate:
            self.integral = proposed_integral
        elif integral_enabled:
            unsaturated = (
                feedforward
                + self.gains.kp * error
                + self.gains.ki * self.integral
                + self.gains.kd * self.filtered_derivative
            )
            saturated = float(np.clip(unsaturated, -limit, limit))

        self.saturated = not math.isclose(unsaturated, saturated, rel_tol=0.0, abs_tol=1e-12)
        return saturated
