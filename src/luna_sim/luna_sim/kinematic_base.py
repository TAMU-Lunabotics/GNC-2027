"""
Kinematic diff-drive robot for the Level 1 sim.

Reads /cmd_vel, integrates the TRUE robot pose in the map frame, publishes ground truth.
Starts at the start pose from the arena layout JSON.
"""
import rclpy
from rclpy.node import Node

import math

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped

from luna_sim.arena.layout import load_layout


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quaternion(yaw):
    """(x, y, z, w) for a rotation about z only. All the 2D sim needs."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def clamp(v, limit):
    return max(-limit, min(limit, v))


def step_diff_drive(x, y, yaw, v, w, dt):
    mid = yaw + 0.5 * w * dt
    return (x + v * math.cos(mid) * dt, y + v * math.sin(mid) * dt, wrap_angle(yaw + w * dt))


class KinematicBase(Node):
    def __init__(self):
        super().__init__('kinematic_base')

        layout_path = self.declare_parameter('arena_json', '').value
        if not layout_path:
            raise RuntimeError("parameter 'arena_json' is required (path to arena_<seed>.json)")
        self.max_v = float(self.declare_parameter('max_linear_speed', 0.5).value)
        self.max_w = float(self.declare_parameter('max_angular_speed', 1.0).value)
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout_s', 0.5).value)

        # State: true pose in the map frame
        start = load_layout(layout_path)['start_pose']
        self.x, self.y, self.yaw = float(start['x']), float(start['y']), float(start['yaw'])
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = None
        self.last_step_time = None

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/sim/ground_truth/pose', 10)
        # Subscribers
        self.twist_pub = self.create_publisher(TwistStamped, '/sim/ground_truth/twist', 10)
        
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 10)

        self.create_timer(1.0 / 50.0, self.on_step)
        self.get_logger().info(f"start pose ({self.x:.2f}, {self.y:.2f}, yaw {self.yaw:.2f}) from {layout_path}")

    def on_cmd(self, msg):
        self.cmd_v = clamp(msg.twist.linear.x, self.max_v)
        self.cmd_w = clamp(msg.twist.angular.z, self.max_w)
        self.last_cmd_time = self.get_clock().now()

    def on_step(self):
        now = self.get_clock().now()

        if self.last_step_time is None:
            self.last_step_time = now
            self.publish(now, 0.0, 0.0)
            return
        dt = (now - self.last_step_time).nanoseconds * 1e-9
        self.last_step_time = now
        if dt <= 0.0:
            return

        v, w = self.cmd_v, self.cmd_w

        self.x, self.y, self.yaw = step_diff_drive(self.x, self.y, self.yaw, v, w, dt)
        self.publish(now, v, w)

    def publish(self, now, v, w):
        stamp = now.to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = yaw_to_quaternion(self.yaw)
        self.pose_pub.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = v
        twist.twist.angular.z = w
        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    KinematicBaseNode = KinematicBase()
    try:
        rclpy.spin(KinematicBaseNode)
    except KeyboardInterrupt:
        pass
    finally:
        KinematicBaseNode.destroy_node()
        rclpy.try_shutdown()