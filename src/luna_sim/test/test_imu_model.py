"""Behavioral checks for the ROS-independent mock IMU model."""

import math
import unittest

import numpy as np

from luna_sim.sensors.imu_model import ImuModel, ImuParameters


def parameters(**changes):
    values = dict(
        rate_hz=50.0,
        gyro_noise_density=0.0,
        gyro_bias_init_std=0.0,
        gyro_bias_rw=0.0,
        accel_noise_density=0.0,
        accel_bias_init_std=0.0,
        accel_bias_rw=0.0,
        gyro_range_dps=250.0,
        accel_range_g=4.0,
        seed=42,
    )
    values.update(changes)
    return ImuParameters(**values)


class TestImuModel(unittest.TestCase):
    def model(
        self, mounting=(0.0, 0.0, 0.0), offset=(0.0, 0.0, 0.0),
        include_offset=False, **changes,
    ):
        return ImuModel(
            parameters(**changes), mounting,
            mounting_xyz=offset, include_mounting_offset=include_offset,
        )

    def test_first_input_establishes_baseline(self):
        model = self.model()
        self.assertIsNone(model.update(0, 0.8, 0.2))
        reading = model.update(20_000_000, 0.8, 0.2)
        self.assertEqual(reading.stamp_ns, 20_000_000)
        self.assertAlmostEqual(reading.linear_acceleration[0], 0.0)

    def test_stationary_upright_sensor_reads_positive_gravity(self):
        model = self.model()
        model.update(0, 0.0, 0.0)
        reading = model.update(20_000_000, 0.0, 0.0)
        self.assertEqual(reading.angular_velocity, (0.0, 0.0, 0.0))
        self.assertEqual(reading.linear_acceleration, (0.0, 0.0, 9.81))

    def test_constant_straight_speed_has_no_horizontal_acceleration(self):
        model = self.model()
        model.update(0, 0.4, 0.0)
        reading = model.update(20_000_000, 0.4, 0.0)
        self.assertEqual(reading.linear_acceleration, (0.0, 0.0, 9.81))

    def test_forward_acceleration_uses_message_time(self):
        model = self.model()
        model.update(0, 0.0, 0.0)
        reading = model.update(100_000_000, 0.2, 0.0)
        self.assertEqual(reading.linear_acceleration, (2.0, 0.0, 9.81))

    def test_turning_adds_signed_lateral_acceleration(self):
        model = self.model()
        model.update(0, 0.4, -0.5)
        reading = model.update(20_000_000, 0.4, -0.5)
        self.assertEqual(reading.angular_velocity, (0.0, 0.0, -0.5))
        self.assertEqual(reading.linear_acceleration, (0.0, -0.2, 9.81))

    def test_roll_mount_rotates_gyro_and_gravity_into_sensor_frame(self):
        model = self.model(mounting=(math.pi / 2, 0.0, 0.0))
        model.update(0, 0.0, 0.5)
        reading = model.update(20_000_000, 0.0, 0.5)
        np.testing.assert_allclose(reading.angular_velocity, (0.0, 0.5, 0.0), atol=1e-12)
        np.testing.assert_allclose(reading.linear_acceleration, (0.0, 9.81, 0.0), atol=1e-12)

    def test_yaw_mount_rotates_forward_acceleration(self):
        model = self.model(mounting=(0.0, 0.0, math.pi / 2))
        model.update(0, 0.0, 0.0)
        reading = model.update(100_000_000, 0.2, 0.0)
        np.testing.assert_allclose(reading.linear_acceleration, (0.0, -2.0, 9.81), atol=1e-12)

    def test_combined_mount_uses_fixed_axis_rotation_order(self):
        model = self.model(mounting=(math.pi / 2, math.pi / 2, 0.0))
        model.update(0, 0.2, 0.5)
        reading = model.update(100_000_000, 0.4, 0.5)
        np.testing.assert_allclose(reading.angular_velocity, (-0.5, 0.0, 0.0), atol=1e-12)
        np.testing.assert_allclose(reading.linear_acceleration, (-9.81, 2.0, -0.2), atol=1e-12)

    def test_covariances_contain_white_noise_variance_only(self):
        model = self.model(
            rate_hz=25.0, gyro_noise_density=0.2, accel_noise_density=0.3,
            gyro_bias_init_std=0.1, accel_bias_init_std=0.2,
        )
        model.update(0, 0.0, 0.0)
        reading = model.update(40_000_000, 0.0, 0.0)
        np.testing.assert_allclose(
            np.array(reading.angular_velocity_covariance).reshape(3, 3), np.eye(3)
        )
        np.testing.assert_allclose(
            np.array(reading.linear_acceleration_covariance).reshape(3, 3), 2.25 * np.eye(3)
        )

    def test_zero_noise_produces_zero_white_noise_covariance(self):
        model = self.model()
        model.update(0, 0.0, 0.0)
        reading = model.update(20_000_000, 0.0, 0.0)
        self.assertEqual(reading.angular_velocity_covariance, (0.0,) * 9)
        self.assertEqual(reading.linear_acceleration_covariance, (0.0,) * 9)

    def test_same_seed_reproduces_all_sensor_errors(self):
        settings = dict(
            gyro_noise_density=7e-5, gyro_bias_init_std=0.01, gyro_bias_rw=1e-4,
            accel_noise_density=8e-4, accel_bias_init_std=0.05, accel_bias_rw=1e-3,
        )
        a, b = self.model(**settings), self.model(**settings)
        for index in range(25):
            motion = (index * 20_000_000, index * 0.002, 0.1)
            self.assertEqual(a.update(*motion), b.update(*motion))

    def test_different_seeds_produce_different_errors(self):
        a = self.model(seed=1, gyro_noise_density=0.01)
        b = self.model(seed=2, gyro_noise_density=0.01)
        a.update(0, 0.0, 0.0)
        b.update(0, 0.0, 0.0)
        self.assertNotEqual(
            a.update(20_000_000, 0.0, 0.0).angular_velocity,
            b.update(20_000_000, 0.0, 0.0).angular_velocity,
        )

    def test_startup_bias_persists_without_random_walk(self):
        model = self.model(gyro_bias_init_std=0.1, accel_bias_init_std=0.1)
        model.update(0, 0.0, 0.0)
        first = model.update(20_000_000, 0.0, 0.0)
        self.assertNotEqual(first.angular_velocity, (0.0, 0.0, 0.0))
        for index in range(2, 12):
            reading = model.update(index * 20_000_000, 0.0, 0.0)
            self.assertEqual(reading.angular_velocity, first.angular_velocity)
            self.assertEqual(reading.linear_acceleration, first.linear_acceleration)

    def test_random_walk_scales_with_square_root_of_elapsed_time(self):
        a = self.model(gyro_bias_rw=0.1, accel_bias_rw=0.1)
        b = self.model(gyro_bias_rw=0.1, accel_bias_rw=0.1)
        a.update(0, 0.0, 0.0)
        b.update(0, 0.0, 0.0)
        short = a.update(10_000_000, 0.0, 0.0)
        long = b.update(40_000_000, 0.0, 0.0)
        np.testing.assert_allclose(long.angular_velocity, 2 * np.array(short.angular_velocity))
        gravity = np.array([0.0, 0.0, 9.81])
        np.testing.assert_allclose(
            np.array(long.linear_acceleration) - gravity,
            2 * (np.array(short.linear_acceleration) - gravity), atol=1e-14,
        )

    def test_white_noise_statistics_match_configured_density(self):
        model = self.model(gyro_noise_density=0.01, accel_noise_density=0.02)
        model.update(0, 0.0, 0.0)
        samples = [model.update(i * 20_000_000, 0.0, 0.0) for i in range(1, 4001)]
        gyro = np.array([r.angular_velocity for r in samples])
        accel = np.array([r.linear_acceleration for r in samples]) - (0.0, 0.0, 9.81)
        for readings, expected_std in (
            (gyro, 0.01 * math.sqrt(50)),
            (accel, 0.02 * math.sqrt(50)),
        ):
            self.assertTrue(np.all(np.abs(readings.mean(axis=0)) < 0.1 * expected_std))
            np.testing.assert_allclose(readings.std(axis=0), expected_std, rtol=0.06)

    def test_full_scale_limits_clip_each_axis_in_si_units(self):
        model = self.model(gyro_range_dps=180.0, accel_range_g=2.0)
        model.update(0, 0.0, 0.0)
        positive = model.update(100_000_000, 4.0, 10.0)
        negative_x = model.update(200_000_000, -4.0, 10.0)
        negative_y = model.update(300_000_000, 4.0, -10.0)
        self.assertEqual(positive.angular_velocity[2], math.pi)
        self.assertEqual(positive.linear_acceleration, (19.62, 19.62, 9.81))
        self.assertEqual(negative_x.linear_acceleration[0], -19.62)
        self.assertEqual(negative_y.angular_velocity[2], -math.pi)
        self.assertEqual(negative_y.linear_acceleration[1], -19.62)

    def test_duplicate_and_older_messages_do_not_change_state_or_rng(self):
        settings = dict(gyro_noise_density=0.01, gyro_bias_rw=0.01)
        a, reference = self.model(**settings), self.model(**settings)
        for model in (a, reference):
            model.update(0, 0.0, 0.0)
            model.update(1_000_000_000, 0.1, 0.0)
        self.assertIsNone(a.update(1_000_000_000, 99.0, 99.0))
        self.assertIsNone(a.update(500_000_000, 88.0, 88.0))
        self.assertEqual(
            a.update(2_000_000_000, 0.2, 0.0),
            reference.update(2_000_000_000, 0.2, 0.0),
        )

    def test_invalid_motion_does_not_advance_state(self):
        model, reference = self.model(), self.model()
        for item in (model, reference):
            item.update(0, 0.0, 0.0)
        for speed, yaw_rate in ((math.nan, 0.0), (0.0, math.inf)):
            with self.assertRaises(ValueError):
                model.update(20_000_000, speed, yaw_rate)
        self.assertEqual(
            model.update(40_000_000, 0.2, 0.0),
            reference.update(40_000_000, 0.2, 0.0),
        )

    def test_invalid_parameters_are_rejected(self):
        invalid = (
            {'rate_hz': 0}, {'rate_hz': 51}, {'rate_hz': math.nan},
            {'gyro_noise_density': -0.1}, {'accel_noise_density': math.inf},
            {'gyro_bias_init_std': -0.1}, {'accel_bias_init_std': math.nan},
            {'gyro_bias_rw': -1.0}, {'accel_bias_rw': math.inf},
            {'gyro_range_dps': 0}, {'accel_range_g': -1},
            {'seed': -1}, {'seed': 1.5}, {'seed': True},
        )
        for setting in invalid:
            with self.subTest(setting=setting):
                with self.assertRaises(ValueError):
                    self.model(**setting)

    def test_offset_adds_centripetal_acceleration_at_constant_yaw_rate(self):
        model = self.model(offset=(0.4, -0.2, 0.2), include_offset=True)
        model.update(0, 0.0, 2.0)
        reading = model.update(100_000_000, 0.0, 2.0)
        np.testing.assert_allclose(reading.linear_acceleration, (-1.6, 0.8, 9.81))
        self.assertEqual(reading.angular_velocity, (0.0, 0.0, 2.0))

    def test_offset_adds_tangential_acceleration_when_yaw_rate_changes(self):
        model = self.model(offset=(0.3, 0.2, 0.2), include_offset=True)
        model.update(0, 0.0, 0.0)
        reading = model.update(100_000_000, 0.0, 0.2)
        np.testing.assert_allclose(reading.linear_acceleration, (-0.412, 0.592, 9.81))

    def test_offset_acceleration_combines_with_translating_base_motion(self):
        model = self.model(offset=(0.2, -0.1, 0.4), include_offset=True)
        model.update(0, 0.2, 0.1)
        reading = model.update(100_000_000, 0.4, 0.3)
        np.testing.assert_allclose(reading.linear_acceleration, (2.182, 0.529, 9.81))

    def test_offset_acceleration_is_rotated_into_the_sensor_frame(self):
        model = self.model(
            mounting=(0.0, 0.0, math.pi / 2),
            offset=(0.4, -0.2, 0.2), include_offset=True,
        )
        model.update(0, 0.0, 2.0)
        reading = model.update(100_000_000, 0.0, 2.0)
        np.testing.assert_allclose(reading.linear_acceleration, (0.8, 1.6, 9.81))

    def test_centered_and_vertical_offsets_preserve_seeded_baseline_readings(self):
        settings = dict(
            gyro_noise_density=7e-5, gyro_bias_init_std=0.01, gyro_bias_rw=1e-4,
            accel_noise_density=8e-4, accel_bias_init_std=0.05, accel_bias_rw=1e-3,
        )
        for offset in ((0.0, 0.0, 0.0), (0.0, 0.0, 0.2)):
            with self.subTest(offset=offset):
                baseline = self.model(**settings)
                extended = self.model(offset=offset, include_offset=True, **settings)
                for index in range(20):
                    motion = (index * 20_000_000, index * 0.002, index * 0.01)
                    self.assertEqual(baseline.update(*motion), extended.update(*motion))

    def test_disabling_offset_effects_preserves_the_original_model(self):
        settings = dict(gyro_noise_density=0.01, accel_noise_density=0.02)
        baseline = self.model(**settings)
        disabled = self.model(offset=(0.4, -0.2, 0.2), include_offset=False, **settings)
        for index in range(20):
            motion = (index * 20_000_000, index * 0.002, index * 0.01)
            self.assertEqual(baseline.update(*motion), disabled.update(*motion))

    def test_rejected_inputs_preserve_offset_yaw_history_and_random_sequence(self):
        settings = dict(
            offset=(0.4, -0.2, 0.2), include_offset=True,
            gyro_noise_density=0.01, accel_noise_density=0.02,
            gyro_bias_rw=0.01, accel_bias_rw=0.01,
        )
        actual, reference = self.model(**settings), self.model(**settings)
        for model in (actual, reference):
            model.update(0, 0.1, 0.2)
            model.update(100_000_000, 0.2, 0.3)
        self.assertIsNone(actual.update(100_000_000, 1.0, 2.0))
        self.assertIsNone(actual.update(50_000_000, 1.0, 2.0))
        for speed, yaw_rate in ((math.nan, 0.4), (0.2, math.inf), (0.2, 1e308)):
            with self.assertRaises(ValueError):
                actual.update(200_000_000, speed, yaw_rate)
        self.assertEqual(
            actual.update(600_000_000, 0.4, 0.5),
            reference.update(600_000_000, 0.4, 0.5),
        )

    def test_invalid_mounting_offsets_and_switches_are_rejected(self):
        for offset in ((0.0, 0.0), (math.nan, 0.0, 0.0), (0.0, math.inf, 0.0)):
            with self.assertRaises(ValueError):
                self.model(offset=offset, include_offset=True)
        for switch in (1, 'true', None):
            with self.assertRaises(ValueError):
                self.model(include_offset=switch)

    def test_invalid_mounting_and_timestamps_are_rejected(self):
        for mounting in ((0.0, 0.0), (math.nan, 0.0, 0.0)):
            with self.assertRaises(ValueError):
                self.model(mounting=mounting)
        model = self.model()
        for stamp in (-1, 1.5, True):
            with self.assertRaises(ValueError):
                model.update(stamp, 0.0, 0.0)


if __name__ == '__main__':
    unittest.main()
