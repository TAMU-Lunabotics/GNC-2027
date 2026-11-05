"""ROS wrapper and topic-delivery checks for the mock IMU."""

import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Imu
import yaml

from luna_sim.sensors.mock_imu import MockImu


class TestMockImuNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[
            '--ros-args',
            '-r', '/imu/data_raw:=/mock_imu_test/data',
            '-r', '/sim/ground_truth/twist:=/mock_imu_test/truth',
        ])

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.geometry = Path(self.temporary.name) / 'geometry.yaml'
        self.geometry.write_text(
            'sensors:\n  imu_link:\n    xyz: [0.0, 0.0, 0.0]\n'
            '    rpy: [0.0, 0.0, 0.0]\n'
        )
        self.node = self.make_node()
        self.real_publisher = self.node.imu_pub
        self.node.imu_pub = Mock()

    def make_node(self, **overrides):
        values = {'geometry_yaml': str(self.geometry)}
        for name in (
            'gyro_noise_density', 'gyro_bias_init_std', 'gyro_bias_rw',
            'accel_noise_density', 'accel_bias_init_std', 'accel_bias_rw',
        ):
            values[name] = 0.0
        values.update(overrides)
        # These are explicit test assumptions for ROS covariance metadata.
        for sensor, variance in (('gyro', 0.04), ('accel', 0.09)):
            if values[f'{sensor}_noise_density'] == 0.0:
                values.setdefault(f'{sensor}_variance_override', variance)
        node = MockImu(parameter_overrides=[
            Parameter(name, value=value) for name, value in values.items()
        ])
        self.addCleanup(node.destroy_node)
        return node

    def twist(self, stamp_ns, speed=0.0, yaw_rate=0.0):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.twist.linear.x = speed
        msg.twist.angular.z = yaw_rate
        return msg

    def prime_and_update(self, speed=0.0, yaw_rate=0.0):
        self.node.on_twist(self.twist(0))
        self.node.on_twist(self.twist(100_000_000, speed, yaw_rate))
        self.node.publish_latest()
        return self.node.imu_pub.publish.call_args.args[0]

    def test_stationary_message_frame_time_and_no_orientation(self):
        msg = self.prime_and_update()
        self.assertEqual(msg.header.frame_id, 'imu_link')
        self.assertEqual(msg.header.stamp.sec, 0)
        self.assertEqual(msg.header.stamp.nanosec, 100_000_000)
        self.assertEqual((msg.orientation.x, msg.orientation.y,
                          msg.orientation.z, msg.orientation.w), (0.0,) * 4)
        self.assertEqual(msg.orientation_covariance[0], -1.0)
        self.assertEqual(msg.angular_velocity.z, 0.0)
        self.assertEqual(msg.linear_acceleration.z, 9.81)

    def test_acceleration_and_yaw_rate_are_converted_to_message_fields(self):
        msg = self.prime_and_update(speed=0.2, yaw_rate=0.5)
        self.assertAlmostEqual(msg.linear_acceleration.x, 2.0)
        self.assertAlmostEqual(msg.linear_acceleration.y, 0.1)
        self.assertAlmostEqual(msg.angular_velocity.z, 0.5)

    def test_no_message_before_second_valid_truth_input(self):
        self.node.publish_latest()
        self.node.on_twist(self.twist(0))
        self.node.publish_latest()
        self.node.imu_pub.publish.assert_not_called()

    def test_timer_does_not_republish_old_readings(self):
        self.prime_and_update()
        self.node.publish_latest()
        self.node.publish_latest()
        self.assertEqual(self.node.imu_pub.publish.call_count, 1)

    def test_latest_measurement_is_selected_when_inputs_are_faster_than_timer(self):
        self.node.on_twist(self.twist(0))
        self.node.on_twist(self.twist(20_000_000, yaw_rate=0.1))
        self.node.on_twist(self.twist(40_000_000, yaw_rate=0.2))
        self.node.publish_latest()
        msg = self.node.imu_pub.publish.call_args.args[0]
        self.assertEqual(msg.header.stamp.nanosec, 40_000_000)
        self.assertEqual(msg.angular_velocity.z, 0.2)

    def test_bad_frame_clears_unpublished_reading_and_recovers(self):
        self.node.on_twist(self.twist(0))
        self.node.on_twist(self.twist(20_000_000))
        bad = self.twist(40_000_000)
        bad.header.frame_id = 'map'
        self.node.on_twist(bad)
        self.node.publish_latest()
        self.node.imu_pub.publish.assert_not_called()
        self.node.on_twist(self.twist(60_000_000))
        self.node.publish_latest()
        self.assertEqual(self.node.imu_pub.publish.call_count, 1)

    def test_nonfinite_motion_and_invalid_stamp_are_rejected(self):
        self.node.on_twist(self.twist(0))
        for bad in (self.twist(20_000_000, speed=math.nan), self.twist(-1)):
            self.node.on_twist(bad)
        bad = self.twist(20_000_000)
        bad.header.stamp.nanosec = 1_000_000_000
        self.node.on_twist(bad)
        self.node.publish_latest()
        self.node.imu_pub.publish.assert_not_called()

    def test_duplicate_and_out_of_order_inputs_are_skipped(self):
        self.prime_and_update()
        self.node.on_twist(self.twist(100_000_000, speed=1.0))
        self.node.on_twist(self.twist(50_000_000, speed=1.0))
        self.node.publish_latest()
        self.assertEqual(self.node.imu_pub.publish.call_count, 1)

    def test_covariance_and_rate_follow_parameter_overrides(self):
        node = self.make_node(
            rate_hz=25.0, gyro_noise_density=0.01, accel_noise_density=0.02
        )
        node.imu_pub = Mock()
        node.on_twist(self.twist(0))
        node.on_twist(self.twist(40_000_000))
        node.publish_latest()
        msg = node.imu_pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.angular_velocity_covariance[0], 0.0025)
        self.assertAlmostEqual(msg.linear_acceleration_covariance[0], 0.01)
        self.assertEqual(node.timer.timer_period_ns, 40_000_000)

    def test_mounting_rotation_is_loaded_from_geometry_yaml(self):
        self.geometry.write_text(
            'sensors:\n  imu_link:\n    xyz: [0.0, 0.0, 0.0]\n'
            '    rpy: [1.5707963267948966, 0.0, 0.0]\n'
        )
        node = self.make_node()
        node.imu_pub = Mock()
        node.on_twist(self.twist(0))
        node.on_twist(self.twist(20_000_000, yaw_rate=0.5))
        node.publish_latest()
        msg = node.imu_pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.angular_velocity.y, 0.5)
        self.assertAlmostEqual(msg.linear_acceleration.y, 9.81)

    def test_startup_parameters_reject_runtime_changes(self):
        changes = {
            'rate_hz': 25.0,
            'gyro_noise_density': 0.01,
            'gyro_bias_init_std': 0.01,
            'gyro_bias_rw': 0.01,
            'accel_noise_density': 0.01,
            'accel_bias_init_std': 0.01,
            'accel_bias_rw': 0.01,
            'gyro_range_dps': 500.0,
            'accel_range_g': 8.0,
            'seed': 43,
            'gyro_variance_override': 0.16,
            'accel_variance_override': 0.25,
            'geometry_yaml': str(self.geometry) + '.different',
            'include_mounting_offset': False,
        }
        for name, value in changes.items():
            with self.subTest(parameter=name):
                original = self.node.get_parameter(name).value
                self.assertTrue(self.node.describe_parameter(name).read_only)
                result, = self.node.set_parameters([Parameter(name, value=value)])
                self.assertFalse(result.successful)
                self.assertEqual(self.node.get_parameter(name).value, original)
        self.assertEqual(self.node.timer.timer_period_ns, 20_000_000)

        # The ROS-managed clock parameter retains its normal dynamic behavior.
        self.assertFalse(self.node.describe_parameter('use_sim_time').read_only)
        result, = self.node.set_parameters([Parameter('use_sim_time', value=True)])
        self.assertTrue(result.successful)
        self.assertTrue(self.node.get_parameter('use_sim_time').value)

    def test_zero_noise_requires_explicit_reported_variance(self):
        for sensor in ('gyro', 'accel'):
            name = f'{sensor}_variance_override'
            with self.subTest(sensor=sensor):
                with self.assertRaisesRegex(ValueError, name):
                    self.make_node(**{name: -1.0})

    def test_zero_noise_keeps_exact_readings_with_explicit_reported_variances(self):
        msg = self.prime_and_update(speed=0.2, yaw_rate=0.5)
        self.assertEqual((msg.angular_velocity.x, msg.angular_velocity.y,
                          msg.angular_velocity.z), (0.0, 0.0, 0.5))
        self.assertEqual((msg.linear_acceleration.x, msg.linear_acceleration.y,
                          msg.linear_acceleration.z), (2.0, 0.1, 9.81))
        for covariance, variance in (
            (msg.angular_velocity_covariance, 0.04),
            (msg.linear_acceleration_covariance, 0.09),
        ):
            for index, value in enumerate(covariance):
                self.assertEqual(value, variance if index in (0, 4, 8) else 0.0)

    def test_reported_variance_overrides_do_not_change_noisy_readings(self):
        settings = {'seed': 17, 'gyro_noise_density': 0.01, 'accel_noise_density': 0.02}
        automatic = self.make_node(**settings)
        overridden = self.make_node(
            **settings, gyro_variance_override=0.04, accel_variance_override=0.09
        )
        for node in (automatic, overridden):
            node.imu_pub = Mock()
            node.on_twist(self.twist(0))
        for index in range(1, 6):
            messages = []
            for node in (automatic, overridden):
                node.on_twist(self.twist(index * 20_000_000, speed=0.2, yaw_rate=0.5))
                node.publish_latest()
                messages.append(node.imu_pub.publish.call_args.args[0])
            self.assertEqual(messages[0].angular_velocity, messages[1].angular_velocity)
            self.assertEqual(messages[0].linear_acceleration, messages[1].linear_acceleration)
            self.assertAlmostEqual(messages[0].angular_velocity_covariance[0], 0.005)
            self.assertAlmostEqual(messages[0].linear_acceleration_covariance[0], 0.02)
            self.assertEqual(messages[1].angular_velocity_covariance[0], 0.04)
            self.assertEqual(messages[1].linear_acceleration_covariance[0], 0.09)

    def test_invalid_reported_variance_overrides_are_rejected(self):
        for name in ('gyro_variance_override', 'accel_variance_override'):
            for value in (0.0, -0.5, -2.0, math.nan, math.inf):
                with self.subTest(parameter=name, value=value):
                    with self.assertRaisesRegex(ValueError, name):
                        self.make_node(**{name: value})
        with self.assertRaisesRegex(ValueError, 'gyro_variance_override'):
            self.make_node(gyro_noise_density=1e-300)

    def test_installed_yaml_has_ros_parameter_key_and_expected_settings(self):
        config = Path(get_package_share_directory('luna_sim')) / 'config/sensors/mock_imu.yaml'
        data = yaml.safe_load(config.read_text())['mock_imu']['ros__parameters']
        self.assertEqual(data['rate_hz'], 50.0)
        self.assertEqual(data['gyro_noise_density'], 7e-5)
        self.assertEqual(data['accel_noise_density'], 8e-4)
        self.assertEqual(data['gyro_variance_override'], -1.0)
        self.assertEqual(data['accel_variance_override'], -1.0)
        self.assertTrue(data['include_mounting_offset'])

    def test_invalid_rate_and_missing_mounting_are_rejected(self):
        with self.assertRaises(ValueError):
            self.make_node(rate_hz=51.0)
        self.geometry.write_text('sensors: {}\n')
        with self.assertRaises(ValueError):
            self.make_node()

    def test_backward_ros_clock_jump_clears_measurement_history(self):
        node = self.make_node(use_sim_time=True)
        node.imu_pub = Mock()
        node.get_clock().set_ros_time_override(Time(seconds=2, clock_type=ClockType.ROS_TIME))
        node.on_twist(self.twist(1_000_000_000))
        node.on_twist(self.twist(2_000_000_000))
        node.get_clock().set_ros_time_override(Time(seconds=0, clock_type=ClockType.ROS_TIME))
        node.publish_latest()
        node.imu_pub.publish.assert_not_called()
        node.on_twist(self.twist(0))
        node.on_twist(self.twist(20_000_000))
        node.publish_latest()
        self.assertEqual(node.imu_pub.publish.call_count, 1)

    def test_actual_ros_input_and_output_topic_delivery(self):
        self.node.imu_pub = self.real_publisher
        observer = rclpy.create_node('mock_imu_test_observer')
        self.addCleanup(observer.destroy_node)
        received = []
        observer.create_subscription(Imu, '/imu/data_raw', received.append, 10)
        truth_pub = observer.create_publisher(TwistStamped, '/sim/ground_truth/twist', 10)
        executor = SingleThreadedExecutor()
        self.addCleanup(executor.shutdown)
        executor.add_node(self.node)
        executor.add_node(observer)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and (
            truth_pub.get_subscription_count() == 0
            or self.real_publisher.get_subscription_count() == 0
        ):
            executor.spin_once(timeout_sec=0.02)
        self.assertGreater(truth_pub.get_subscription_count(), 0)
        self.assertGreater(self.real_publisher.get_subscription_count(), 0)
        truth_pub.publish(self.twist(0))
        truth_pub.publish(self.twist(20_000_000, yaw_rate=0.5))
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        self.assertTrue(received, 'No IMU message arrived over ROS topics.')
        self.assertEqual(received[0].header.frame_id, 'imu_link')
        self.assertEqual(received[0].angular_velocity.z, 0.5)

    def test_mounting_xyz_is_loaded_and_offset_acceleration_is_published(self):
        self.geometry.write_text(
            'sensors:\n  imu_link:\n    xyz: [0.4, -0.2, 0.2]\n'
            '    rpy: [0.0, 0.0, 0.0]\n'
        )
        node = self.make_node()
        node.imu_pub = Mock()
        self.assertTrue(node.get_parameter('include_mounting_offset').value)
        node.on_twist(self.twist(0, yaw_rate=2.0))
        node.on_twist(self.twist(100_000_000, yaw_rate=2.0))
        node.publish_latest()
        msg = node.imu_pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.linear_acceleration.x, -1.6)
        self.assertAlmostEqual(msg.linear_acceleration.y, 0.8)
        self.assertAlmostEqual(msg.linear_acceleration.z, 9.81)
        self.assertEqual(msg.angular_velocity.z, 2.0)

    def test_mounting_offset_switch_preserves_original_readings(self):
        self.geometry.write_text(
            'sensors:\n  imu_link:\n    xyz: [0.4, -0.2, 0.2]\n'
            '    rpy: [0.0, 0.0, 0.0]\n'
        )
        node = self.make_node(include_mounting_offset=False)
        node.imu_pub = Mock()
        node.on_twist(self.twist(0, yaw_rate=2.0))
        node.on_twist(self.twist(100_000_000, yaw_rate=2.0))
        node.publish_latest()
        msg = node.imu_pub.publish.call_args.args[0]
        self.assertEqual((msg.linear_acceleration.x, msg.linear_acceleration.y,
                          msg.linear_acceleration.z), (0.0, 0.0, 9.81))

    def test_clock_reset_preserves_mount_and_clears_offset_yaw_history(self):
        self.geometry.write_text(
            'sensors:\n  imu_link:\n    xyz: [0.4, 0.1, 0.2]\n'
            '    rpy: [0.0, 0.0, 0.0]\n'
        )
        node = self.make_node(use_sim_time=True)
        node.imu_pub = Mock()
        node.get_clock().set_ros_time_override(Time(seconds=2, clock_type=ClockType.ROS_TIME))
        node.on_twist(self.twist(0))
        node.on_twist(self.twist(1_000_000_000, yaw_rate=1.0))
        node.get_clock().set_ros_time_override(Time(seconds=0, clock_type=ClockType.ROS_TIME))
        node.on_twist(self.twist(0, yaw_rate=2.0))
        node.publish_latest()
        node.imu_pub.publish.assert_not_called()
        node.on_twist(self.twist(20_000_000, yaw_rate=2.0))
        node.publish_latest()
        msg = node.imu_pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.linear_acceleration.x, -1.6)
        self.assertAlmostEqual(msg.linear_acceleration.y, -0.4)
        self.assertAlmostEqual(msg.linear_acceleration.z, 9.81)

    def test_offset_requires_valid_xyz_and_disabled_mode_accepts_rpy_only(self):
        for xyz in (None, '[0.0, 0.0]', '[.nan, 0.0, 0.0]'):
            position = '' if xyz is None else f'    xyz: {xyz}\n'
            self.geometry.write_text(
                'sensors:\n  imu_link:\n' + position + '    rpy: [0.0, 0.0, 0.0]\n'
            )
            with self.subTest(xyz=xyz):
                with self.assertRaisesRegex(ValueError, 'xyz|XYZ'):
                    self.make_node()
        self.geometry.write_text('sensors:\n  imu_link:\n    rpy: [0.0, 0.0, 0.0]\n')
        node = self.make_node(include_mounting_offset=False)
        self.assertEqual(node.mounting_xyz, (0.0, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
