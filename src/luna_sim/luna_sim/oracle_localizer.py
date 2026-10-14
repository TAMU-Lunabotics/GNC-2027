"""
Publishes the ground-truth robot pose as the robot's actual odometry.

Publishes exactly where and in the same form as the real localization publishes, so everything downstream (pathing, mission, RViz) works before state-estimation exists, or 
whenever something needs to be tested without state-estiation error getting in the way

Never run it alongside the state-estimator, both would publish map -> odom -> base_link.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# Small, non-zero covariance: "very confident", but not the all-zeros that means "unknown".
POSE_VAR = 1e-6      # m^2 / rad^2
TWIST_VAR = 1e-6


class OracleLocalizer(Node):
    def __init__(self):
        super().__init__('oracle_localizer')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)

        self.latest_twist = None

        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(TwistStamped, '/sim/ground_truth/twist', self.on_twist, 10)
        self.create_subscription(PoseStamped, '/sim/ground_truth/pose', self.on_pose, 10)

        if self.publish_tf:
            self.publish_map_to_odom()
        self.get_logger().warn('ORACLE localization: publishing ground truth as the estimate')

    def publish_map_to_odom(self):
        """map -> odom is identity: with perfect localization, odometry never drifts."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    def on_twist(self, msg):
        self.latest_twist = msg.twist

    def on_pose(self, msg):
        if msg.header.frame_id not in ('', self.map_frame):
            self.get_logger().error(
                f"ground truth is in frame '{msg.header.frame_id}', expected '{self.map_frame}'",
                throttle_duration_sec=5.0)
            return
        stamp = msg.header.stamp

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = msg.pose
        odom.pose.covariance = [POSE_VAR if i % 7 == 0 else 0.0 for i in range(36)]
        if self.latest_twist is not None:
            odom.twist.twist = self.latest_twist
        odom.twist.covariance = [TWIST_VAR if i % 7 == 0 else 0.0 for i in range(36)]
        self.odom_pub.publish(odom)

        if self.publish_tf:
            # map == odom here, so the odom -> base_link transform is the map-frame pose.
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = msg.pose.position.x
            t.transform.translation.y = msg.pose.position.y
            t.transform.translation.z = msg.pose.position.z
            t.transform.rotation = msg.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OracleLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()