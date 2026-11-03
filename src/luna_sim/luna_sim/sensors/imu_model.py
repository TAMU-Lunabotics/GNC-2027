"""ROS-independent IMU measurements for the flat Level 1 simulator."""

from dataclasses import dataclass
import math

import numpy as np


GRAVITY_MPS2 = 9.81


@dataclass(frozen=True)
class ImuParameters:
    """Sensor settings supplied by the mock IMU configuration."""

    rate_hz: float
    gyro_noise_density: float
    gyro_bias_init_std: float
    gyro_bias_rw: float
    accel_noise_density: float
    accel_bias_init_std: float
    accel_bias_rw: float
    gyro_range_dps: float
    accel_range_g: float
    seed: int

    def __post_init__(self):
        if not math.isfinite(self.rate_hz) or not 0 < self.rate_hz <= 50:
            raise ValueError('IMU rate must be greater than zero and at most 50 Hz.')

        for name in (
            'gyro_noise_density', 'gyro_bias_init_std', 'gyro_bias_rw',
            'accel_noise_density', 'accel_bias_init_std', 'accel_bias_rw',
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative.')

        for name in ('gyro_range_dps', 'accel_range_g'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive.')
        if type(self.seed) is not int or self.seed < 0:
            raise ValueError('Seed must be a nonnegative integer.')


@dataclass(frozen=True)
class ImuReading:
    """Sensor-frame readings and white-noise covariances in SI units."""

    stamp_ns: int
    angular_velocity: tuple[float, float, float]
    linear_acceleration: tuple[float, float, float]
    angular_velocity_covariance: tuple[float, ...]
    linear_acceleration_covariance: tuple[float, ...]


def _base_to_imu_rotation(mounting_rpy):
    """Invert the fixed-axis mounting rotation Rz(yaw) Ry(pitch) Rx(roll)."""
    if len(mounting_rpy) != 3 or not all(
        math.isfinite(value) for value in mounting_rpy
    ):
        raise ValueError('Mounting RPY must contain three finite angles.')

    roll, pitch, yaw = mounting_rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    imu_to_base = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    return imu_to_base.T


def _diagonal_covariance(variance):
    return (variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance)


class ImuModel:
    """
    Generate samples from stamped forward speed and yaw rate.

    The first input establishes the velocity/time baseline and returns None.
    Duplicate or older stamps return None without advancing state or the RNG.
    Optional mounting-offset effects use planar rigid-body acceleration.
    """

    def __init__(
        self, parameters: ImuParameters, mounting_rpy,
        mounting_xyz=(0.0, 0.0, 0.0), include_mounting_offset=False,
    ):
        self._parameters = parameters
        self._rotation = _base_to_imu_rotation(mounting_rpy)
        if len(mounting_xyz) != 3 or not all(math.isfinite(v) for v in mounting_xyz):
            raise ValueError('Mounting XYZ must contain three finite positions in meters.')
        if type(include_mounting_offset) is not bool:
            raise ValueError('include_mounting_offset must be Boolean.')
        self._mounting_xyz = tuple(mounting_xyz)
        self._include_mounting_offset = include_mounting_offset
        self._gyro_std = parameters.gyro_noise_density * math.sqrt(parameters.rate_hz)
        self._accel_std = parameters.accel_noise_density * math.sqrt(parameters.rate_hz)
        self._gyro_limit = math.radians(parameters.gyro_range_dps)
        self._accel_limit = parameters.accel_range_g * GRAVITY_MPS2
        gyro_variance = self._gyro_std * self._gyro_std
        accel_variance = self._accel_std * self._accel_std
        if not all(math.isfinite(value) for value in (
            self._gyro_limit, self._accel_limit,
            gyro_variance, accel_variance,
        )):
            raise ValueError('Sensor settings produce nonfinite limits or variances.')

        self._gyro_covariance = _diagonal_covariance(gyro_variance)
        self._accel_covariance = _diagonal_covariance(accel_variance)
        self._rng = np.random.Generator(np.random.PCG64(parameters.seed))
        self._gyro_bias = self._draw(parameters.gyro_bias_init_std)
        self._accel_bias = self._draw(parameters.accel_bias_init_std)
        self._previous_stamp_ns = None
        self._previous_speed = None
        self._previous_yaw_rate = None

    def _draw(self, standard_deviation):
        if standard_deviation == 0:
            return np.zeros(3)
        return self._rng.normal(0.0, standard_deviation, size=3)

    def update(self, stamp_ns, forward_speed_mps, yaw_rate_radps):
        """Return the next measurement, or None when no positive dt exists."""
        if type(stamp_ns) is not int or stamp_ns < 0:
            raise ValueError('Timestamp must be a nonnegative integer in nanoseconds.')
        if not all(math.isfinite(value) for value in (
            forward_speed_mps, yaw_rate_radps,
        )):
            raise ValueError('Speed and yaw rate must be finite.')

        if self._previous_stamp_ns is None:
            self._previous_stamp_ns = stamp_ns
            self._previous_speed = forward_speed_mps
            self._previous_yaw_rate = yaw_rate_radps
            return None
        if stamp_ns <= self._previous_stamp_ns:
            return None

        dt = (stamp_ns - self._previous_stamp_ns) / 1_000_000_000
        acceleration = (forward_speed_mps - self._previous_speed) / dt
        lateral_acceleration = forward_speed_mps * yaw_rate_radps
        offset_x, offset_y, _ = self._mounting_xyz
        if self._include_mounting_offset and (offset_x != 0.0 or offset_y != 0.0):
            yaw_acceleration = (yaw_rate_radps - self._previous_yaw_rate) / dt
            # alpha x r + omega x (omega x r), for planar rotation about z.
            acceleration -= (
                yaw_acceleration * offset_y
                + yaw_rate_radps * (yaw_rate_radps * offset_x)
            )
            lateral_acceleration += (
                yaw_acceleration * offset_x
                - yaw_rate_radps * (yaw_rate_radps * offset_y)
            )
        if not math.isfinite(acceleration) or not math.isfinite(lateral_acceleration):
            raise ValueError('Motion produces nonfinite acceleration.')

        angular_velocity = self._rotation @ np.array([0.0, 0.0, yaw_rate_radps])
        specific_force = self._rotation @ np.array([
            acceleration, lateral_acceleration, GRAVITY_MPS2,
        ])

        # A random walk coefficient has units per sqrt(second).
        root_dt = math.sqrt(dt)
        self._gyro_bias += self._draw(self._parameters.gyro_bias_rw * root_dt)
        self._accel_bias += self._draw(self._parameters.accel_bias_rw * root_dt)
        angular_velocity += self._gyro_bias + self._draw(self._gyro_std)
        specific_force += self._accel_bias + self._draw(self._accel_std)

        angular_velocity = np.clip(angular_velocity, -self._gyro_limit, self._gyro_limit)
        specific_force = np.clip(specific_force, -self._accel_limit, self._accel_limit)
        self._previous_stamp_ns = stamp_ns
        self._previous_speed = forward_speed_mps
        self._previous_yaw_rate = yaw_rate_radps

        return ImuReading(
            stamp_ns=stamp_ns,
            angular_velocity=tuple(float(value) for value in angular_velocity),
            linear_acceleration=tuple(float(value) for value in specific_force),
            angular_velocity_covariance=self._gyro_covariance,
            linear_acceleration_covariance=self._accel_covariance,
        )
