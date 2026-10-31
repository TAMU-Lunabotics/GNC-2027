"""Simulated wheel-encoder odometry for the Level 1 rover simulation."""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import yaml

from luna_sim.sensors.wheel_model import WheelOdometryModel
from luna_sim.sensors.wheel_model import expand_covariance


class MockWheelOdometry(Node):
    def __init__(self):
        super().__init__('mock_wheel_odometry')

        geometry_path = self.declare_parameter('geometry_yaml', '').value
        if not geometry_path:
            raise ValueError('geometry_yaml must point to robot_geometry.yaml')
        with open(geometry_path, encoding='utf-8') as stream:
            geometry = yaml.safe_load(stream)
        wheels = geometry['wheels']

        rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)
        ticks_per_rev = int(self.declare_parameter('ticks_per_rev', 4096).value)
        noise_std = float(self.declare_parameter('noise_std_rad_s', 0.01).value)
        noise_per_rad_s = float(self.declare_parameter('noise_per_rad_s', 0.01).value)
        seed = int(self.declare_parameter('seed', 0).value)
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError('rate_hz must be finite and positive')

        self.model = WheelOdometryModel(
            wheel_radius=float(wheels['radius']),
            wheel_separation=float(wheels['separation']),
            ticks_per_rev=ticks_per_rev,
            noise_std=noise_std,
            noise_per_rad_s=noise_per_rad_s,
            seed=seed,
        )
        self.rate_hz = rate_hz
        self.max_age_ns = int(3.0 / rate_hz * 1e9)
        self.last_input_ns = None
        self.last_step_ns = None
        self.true_v = 0.0
        self.true_w = 0.0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Odometry, '/wheel_odom', qos)
        self.subscription = self.create_subscription(
            TwistStamped, '/sim/ground_truth/twist', self.on_twist, qos)
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(
            f'Mock wheel odometry: radius={wheels["radius"]} m, '
            f'separation={wheels["separation"]} m, seed={seed}'
        )

    def on_twist(self, msg):
        if msg.header.frame_id != 'base_link':
            self.get_logger().warning('Ignoring twist not in base_link frame')
            return
        v = float(msg.twist.linear.x)
        w = float(msg.twist.angular.z)
        if not math.isfinite(v) or not math.isfinite(w):
            self.get_logger().warning('Ignoring non-finite ground-truth twist')
            return
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.last_input_ns is not None and stamp_ns < self.last_input_ns:
            # Simulation time jumped backwards; reset the odometry epoch.
            self.model.reset()
            self.last_step_ns = None
        self.last_input_ns = stamp_ns
        self.true_v, self.true_w = v, w

    def on_timer(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if self.last_input_ns is None or now_ns <= 0:
            return
        # Do not integrate stale ground truth or integrate over a time jump.
        age_ns = now_ns - self.last_input_ns
        if age_ns < 0 or age_ns > self.max_age_ns:
            self.last_step_ns = None
            return
        if self.last_step_ns is None:
            self.last_step_ns = now_ns
            return
        dt = (now_ns - self.last_step_ns) * 1e-9
        self.last_step_ns = now_ns
        if not math.isfinite(dt) or dt <= 0:
            return

        sample = self.model.step(self.true_v, self.true_w, dt)
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = sample.x
        msg.pose.pose.position.y = sample.y
        msg.pose.pose.orientation.z = math.sin(sample.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(sample.yaw / 2.0)
        msg.twist.twist.linear.x = sample.linear_velocity
        msg.twist.twist.angular.z = sample.angular_velocity
        msg.pose.covariance = expand_covariance(self.model.pose_covariance, (0, 1, 5))
        msg.twist.covariance = expand_covariance(self.model.twist_covariance, (0, 5))
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockWheelOdometry()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
