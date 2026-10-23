"""ROS wrapper around the path follower: /guidance/path + /odometry/filtered -> /cmd_vel."""
import math
from dataclasses import fields

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped, TwistStamped
from luna_interfaces.msg import GuidanceStatus
from nav_msgs.msg import Odometry, Path

from luna_guidance import pure_pursuit as pp

STATE_TO_MSG = {
    pp.IDLE: GuidanceStatus.IDLE,
    pp.FOLLOWING: GuidanceStatus.FOLLOWING,
    pp.GOAL_REACHED: GuidanceStatus.GOAL_REACHED,
    pp.STOPPED: GuidanceStatus.STOPPED,
}

STATUS_RATE_HZ = 5.0


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def is_valid_quaternion(q):
    return (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 1e-6


class GuidanceControllerNode(Node):
    def __init__(self):
        super().__init__('guidance_controller')

        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        self.odom_timeout_s = float(self.declare_parameter('odom_timeout_s', 0.5).value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)

        defaults = pp.Params()
        values = {}
        for f in fields(pp.Params):
            default = getattr(defaults, f.name)
            raw = self.declare_parameter(f.name, default).value
            values[f.name] = type(default)(raw)
        self.controller = pp.PurePursuit(pp.Params(**values))

        self.pose = None
        self.pose_time = None
        self.last_tick = None
        self.tick_count = 0
        self.last_status_key = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        debug_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Path, '/guidance/path', self.on_path, latched)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, odom_qos)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', cmd_qos)
        self.status_pub = self.create_publisher(GuidanceStatus, '/guidance/status', latched)
        self.lookahead_pub = self.create_publisher(PointStamped, '/guidance/lookahead', debug_qos)

        self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.publish_status()
        self.get_logger().info(f'guidance controller up: {self.rate_hz:.0f} Hz, '
                               f'lookahead {self.controller.p.lookahead_m} m, '
                               f'cruise {self.controller.p.cruise_speed} m/s')

    def on_path(self, msg):
        if msg.header.frame_id not in ('', self.map_frame):
            self.get_logger().error(f"path is in frame '{msg.header.frame_id}', expected "
                                    f"'{self.map_frame}': ignoring it and stopping")
            self.controller.set_path([])
            return

        points = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        goal_yaw = None
        if len(msg.poses) >= 2 and is_valid_quaternion(msg.poses[-1].pose.orientation):
            goal_yaw = yaw_from_quaternion(msg.poses[-1].pose.orientation)
        self.controller.set_path(points, goal_yaw)
        self.get_logger().info(f'new path: {len(points)} points'
                               if points else 'empty path: stopping')

    def on_odom(self, msg):
        if msg.header.frame_id not in ('', self.map_frame):
            self.get_logger().error(
                f"odometry is in frame '{msg.header.frame_id}', expected '{self.map_frame}'",
                throttle_duration_sec=5.0)
            return
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.pose_time = self.get_clock().now()

    def on_timer(self):
        now = self.get_clock().now()
        dt = 1.0 / self.rate_hz
        if self.last_tick is not None:
            measured = (now - self.last_tick).nanoseconds * 1e-9
            if measured > 0.0:
                dt = measured
        self.last_tick = now

        controller = self.controller
        pose_age = None if self.pose_time is None else (now - self.pose_time).nanoseconds * 1e-9
        stale = pose_age is None or pose_age > self.odom_timeout_s

        if controller.has_path and stale:
            reason = ('no localization yet' if pose_age is None
                      else f'no localization for {pose_age:.1f} s')
            v, w = controller.stop(reason)
        else:
            x, y, yaw = self.pose if self.pose is not None else (0.0, 0.0, 0.0)
            v, w = controller.get_control(x, y, yaw, dt)

        self.publish_cmd(v, w, now)

        if controller.lookahead_point is not None and controller.state == pp.FOLLOWING:
            point = PointStamped()
            point.header.stamp = now.to_msg()
            point.header.frame_id = self.map_frame
            point.point.x, point.point.y = controller.lookahead_point
            self.lookahead_pub.publish(point)

        self.tick_count += 1
        key = (controller.state, controller.reason)
        every = max(int(self.rate_hz / STATUS_RATE_HZ), 1)
        if key != self.last_status_key or self.tick_count % every == 0:
            self.publish_status(now)

    def publish_cmd(self, v, w, now=None):
        cmd = TwistStamped()
        cmd.header.stamp = (now or self.get_clock().now()).to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def publish_status(self, now=None):
        c = self.controller
        status = GuidanceStatus()
        status.header.stamp = (now or self.get_clock().now()).to_msg()
        status.state = STATE_TO_MSG[c.state]
        status.message = c.reason
        status.cross_track_error_m = float(c.cross_track)
        status.distance_to_goal_m = float(c.dist_to_goal)
        self.status_pub.publish(status)
        self.last_status_key = (c.state, c.reason)


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
