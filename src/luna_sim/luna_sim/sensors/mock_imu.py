"""Publish simulated IMU measurements from Level 1 ground-truth motion."""

from dataclasses import fields
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
import yaml

from luna_sim.sensors.imu_model import ImuModel, ImuParameters


def _load_yaml(path):
    """Load a YAML mapping with an actionable configuration error."""
    try:
        with Path(path).open(encoding='utf-8') as source:
            data = yaml.safe_load(source)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'Cannot load YAML {path}: {error}') from error
    if not isinstance(data, dict):
        raise ValueError(f'Expected a YAML mapping in {path}.')
    return data


def _reported_covariance(noise_density, rate_hz, variance_override, parameter_name):
    """Require a positive reported variance, using generated white noise by default."""
    if not math.isfinite(variance_override) or (
        variance_override != -1.0 and variance_override <= 0.0
    ):
        raise ValueError(f'{parameter_name} must be -1.0 (automatic) or positive.')

    standard_deviation = noise_density * math.sqrt(rate_hz)
    variance = (
        standard_deviation * standard_deviation
        if variance_override == -1.0 else variance_override
    )
    if not math.isfinite(variance) or variance <= 0.0:
        raise ValueError(
            f'{parameter_name}: automatic covariance must be positive; '
            'when white noise is disabled, supply an explicit positive '
            'variance for the consuming filter.'
        )
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


class MockImu(Node):
    """Wrap the sensor model with a subscription and ROS-clock timer."""

    def __init__(self, **kwargs):
        super().__init__('mock_imu', **kwargs)
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _declare_startup_parameter(self, name, default):
        """Accept startup overrides and reject later changes to fixed settings."""
        return self.declare_parameter(
            name, default, descriptor=ParameterDescriptor(read_only=True)
        ).value

    def _configure(self):
        config = Path(get_package_share_directory('luna_sim')) / 'config/sensors/mock_imu.yaml'
        try:
            defaults = _load_yaml(config)['mock_imu']['ros__parameters']
            values = {
                item.name: self._declare_startup_parameter(item.name, defaults[item.name])
                for item in fields(ImuParameters)
            }
            variance_overrides = {
                name: self._declare_startup_parameter(name, defaults[name])
                for name in ('gyro_variance_override', 'accel_variance_override')
            }
            self.include_mounting_offset = self._declare_startup_parameter(
                'include_mounting_offset', defaults['include_mounting_offset']
            )
        except (KeyError, TypeError) as error:
            raise ValueError(f'Invalid mock IMU parameter configuration: {config}') from error
        self.sensor_settings = ImuParameters(**values)
        if type(self.include_mounting_offset) is not bool:
            raise ValueError('include_mounting_offset must be Boolean.')
        self._gyro_covariance = _reported_covariance(
            self.sensor_settings.gyro_noise_density, self.sensor_settings.rate_hz,
            variance_overrides['gyro_variance_override'], 'gyro_variance_override',
        )
        self._accel_covariance = _reported_covariance(
            self.sensor_settings.accel_noise_density, self.sensor_settings.rate_hz,
            variance_overrides['accel_variance_override'], 'accel_variance_override',
        )

        geometry_yaml = self._declare_startup_parameter('geometry_yaml', '')
        if not geometry_yaml:
            raise ValueError('geometry_yaml is required.')
        try:
            mounting = _load_yaml(geometry_yaml)['sensors']['imu_link']
            self.mounting_rpy = tuple(mounting['rpy'])
            self.mounting_xyz = tuple(
                mounting['xyz'] if self.include_mounting_offset
                else mounting.get('xyz', (0.0, 0.0, 0.0))
            )
        except (KeyError, TypeError) as error:
            raise ValueError(
                'Geometry must define sensors.imu_link.rpy and xyz '
                'when mounting-offset effects are enabled.'
            ) from error

        self.model = ImuModel(
            self.sensor_settings, self.mounting_rpy,
            mounting_xyz=self.mounting_xyz,
            include_mounting_offset=self.include_mounting_offset,
        )
        self._latest_reading = None
        self._last_published_stamp = None
        self._bad_input_reported = False

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos)
        self.create_subscription(TwistStamped, '/sim/ground_truth/twist', self.on_twist, qos)
        self.timer = self.create_timer(1.0 / self.sensor_settings.rate_hz, self.publish_latest)
        self._jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None,
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=True,
            ),
            post_callback=self.on_clock_jump,
        )
        self.get_logger().info(
            f'Mock IMU: {self.sensor_settings.rate_hz:g} Hz, '
            f'seed={self.sensor_settings.seed}, frame=imu_link'
        )

    def on_twist(self, msg):
        """Update sensor state only from valid, stamped body motion."""
        try:
            if msg.header.frame_id != 'base_link':
                raise ValueError('Ground-truth twist frame must be base_link.')
            stamp = msg.header.stamp
            if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
                raise ValueError('Ground-truth timestamp is invalid.')
            reading = self.model.update(
                stamp.sec * 1_000_000_000 + stamp.nanosec,
                msg.twist.linear.x,
                msg.twist.angular.z,
            )
        except ValueError as error:
            self._latest_reading = None
            if not self._bad_input_reported:
                self.get_logger().warning(f'Ground-truth motion rejected: {error}')
            self._bad_input_reported = True
            return

        self._bad_input_reported = False
        if reading is not None:
            self._latest_reading = reading

    def publish_latest(self):
        """Publish each selected measurement once, preserving its source time."""
        reading = self._latest_reading
        if reading is None or reading.stamp_ns == self._last_published_stamp:
            return

        msg = Imu()
        msg.header.frame_id = 'imu_link'
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            reading.stamp_ns, 1_000_000_000
        )
        # Raw IMU output has no orientation estimate.
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            reading.angular_velocity
        )
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
            reading.linear_acceleration
        )
        msg.angular_velocity_covariance = list(self._gyro_covariance)
        msg.linear_acceleration_covariance = list(self._accel_covariance)
        self.imu_pub.publish(msg)
        self._last_published_stamp = reading.stamp_ns

    def on_clock_jump(self, _jump):
        """Start a fresh seeded measurement sequence after a clock reset."""
        self.model = ImuModel(
            self.sensor_settings, self.mounting_rpy,
            mounting_xyz=self.mounting_xyz,
            include_mounting_offset=self.include_mounting_offset,
        )
        self._latest_reading = None
        self._last_published_stamp = None
        self._bad_input_reported = False

    def destroy_node(self):
        """Release the clock callback before destroying ROS entities."""
        handle = getattr(self, '_jump_handle', None)
        if handle is not None:
            handle.unregister()
            self._jump_handle = None
        return super().destroy_node()


def main(args=None):
    """Run the mock IMU with the single-threaded ROS executor."""
    rclpy.init(args=args)
    node = None
    try:
        node = MockImu()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
