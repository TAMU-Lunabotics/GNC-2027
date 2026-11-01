#!/usr/bin/env python3
"""
Encoder odometry node for a differential / skid-steer drive.

Subscribes:  /encoders        std_msgs/Int32MultiArray  (cumulative ticks)
Publishes:   /wheel/odometry  nav_msgs/Odometry
             odom -> base_link TF (optional, off by default)
Services:    ~/reset          std_srvs/Trigger  (zero the pose)

Encoder order and signs are configurable because the RoboClaw driver publishes
[M1, -M2], which may not be [left, right]. Put the robot on blocks, spin each
wheel FORWARD, and set left_index/right_index/left_sign/right_sign so both
deltas come out positive.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

BIG = 1e6  # covariance for unmeasured dimensions


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def tick_delta(cur: int, prev: int) -> int:
    """Difference of two 32-bit counters, correct across rollover."""
    return ((cur - prev + 2**31) % 2**32) - 2**31


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        # ---------------- parameters ----------------
        p = self.declare_parameter
        p('ticks_per_rev', 0.0)        # encoder ticks per WHEEL revolution (after gearing)
        p('wheel_radius', 0.0)         # m
        p('track_width', 0.0)          # m, EFFECTIVE width (calibrate for skid steer)
        p('left_index', 0)             # index into msg.data for left wheel
        p('right_index', 1)
        p('left_sign', 1)              # +1 or -1 so forward motion gives positive ticks
        p('right_sign', 1)
        p('encoder_topic', '/encoders')
        p('odom_topic', '/wheel/odometry')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('publish_tf', False)         # leave False if an EKF publishes odom->base_link
        p('max_dt', 1.0)               # s, longer gaps skip the velocity estimate
        p('vel_filter_alpha', 0.5)     # 1.0 = no filtering, smaller = smoother
        p('var_vx', 0.005)             # (m/s)^2
        p('var_wz', 0.05)              # (rad/s)^2, skid-steer yaw is poor; fuse an IMU
        p('pose_var_growth', 0.01)     # m^2 of xy variance added per meter traveled
        p('yaw_var_growth', 0.05)      # rad^2 of yaw variance added per rad turned

        g = lambda n: self.get_parameter(n).value
        self.ticks_per_rev = float(g('ticks_per_rev'))
        self.wheel_radius = float(g('wheel_radius'))
        self.track_width = float(g('track_width'))
        bad = [n for n, v in (('ticks_per_rev', self.ticks_per_rev),
                              ('wheel_radius', self.wheel_radius),
                              ('track_width', self.track_width)) if v <= 0.0]
        if bad:
            raise ValueError(f'Parameters must be > 0: {", ".join(bad)}')

        self.left_idx = int(g('left_index'))
        self.right_idx = int(g('right_index'))
        self.left_sign = 1 if int(g('left_sign')) >= 0 else -1
        self.right_sign = 1 if int(g('right_sign')) >= 0 else -1
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.max_dt = float(g('max_dt'))
        self.alpha = min(max(float(g('vel_filter_alpha')), 0.0), 1.0)
        self.var_vx = float(g('var_vx'))
        self.var_wz = float(g('var_wz'))
        self.pose_var_growth = float(g('pose_var_growth'))
        self.yaw_var_growth = float(g('yaw_var_growth'))

        self.m_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev

        # ---------------- state ----------------
        self._reset_state()

        # ---------------- ROS interfaces ----------------
        self.odom_pub = self.create_publisher(Odometry, g('odom_topic'), 10)
        self.tf_broadcaster = TransformBroadcaster(self) if g('publish_tf') else None
        self.create_subscription(Int32MultiArray, g('encoder_topic'),
                                 self.encoder_cb, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset', self.reset_cb)

        self.get_logger().info(
            f'Encoder odometry started: {self.m_per_tick * 1000:.3f} mm/tick, '
            f'track {self.track_width:.3f} m, '
            f'L=data[{self.left_idx}]*{self.left_sign}, '
            f'R=data[{self.right_idx}]*{self.right_sign}, '
            f'TF {"on" if self.tf_broadcaster else "off"}')

    # ------------------------------------------------------------------
    def _reset_state(self):
        self.x = self.y = self.theta = 0.0
        self.var_xy = 0.0
        self.var_yaw = 0.0
        self.v = self.w = 0.0
        self.prev_left = self.prev_right = None
        self.prev_time = None

    def reset_cb(self, _req, resp):
        self._reset_state()
        resp.success = True
        resp.message = 'Odometry pose reset to zero'
        self.get_logger().info(resp.message)
        return resp

    # ------------------------------------------------------------------
    def encoder_cb(self, msg: Int32MultiArray):
        need = max(self.left_idx, self.right_idx) + 1
        if len(msg.data) < need:
            self.get_logger().warn(
                f'Expected >= {need} encoder values, got {len(msg.data)}',
                throttle_duration_sec=5.0)
            return

        left = self.left_sign * int(msg.data[self.left_idx])
        right = self.right_sign * int(msg.data[self.right_idx])
        now = self.get_clock().now()

        if self.prev_time is None:
            self.prev_left, self.prev_right, self.prev_time = left, right, now
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        d_left = tick_delta(left, self.prev_left) * self.m_per_tick
        d_right = tick_delta(right, self.prev_right) * self.m_per_tick
        self.prev_left, self.prev_right, self.prev_time = left, right, now

        if dt <= 0.0:
            return

        # Pose integration (midpoint). Done even after a long gap, since
        # distance traveled doesn't depend on dt; only velocity does.
        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.track_width
        mid = self.theta + 0.5 * d_theta
        self.x += d_center * math.cos(mid)
        self.y += d_center * math.sin(mid)
        self.theta = wrap_angle(self.theta + d_theta)

        # Dead-reckoning uncertainty grows with motion.
        self.var_xy += self.pose_var_growth * abs(d_center)
        self.var_yaw += self.yaw_var_growth * abs(d_theta)

        # Velocity (skip on long gaps, low-pass filter otherwise)
        if dt <= self.max_dt:
            v_raw = d_center / dt
            w_raw = d_theta / dt
            self.v = self.alpha * v_raw + (1.0 - self.alpha) * self.v
            self.w = self.alpha * w_raw + (1.0 - self.alpha) * self.w
        else:
            self.get_logger().warn(
                f'Encoder gap of {dt:.2f}s, pose integrated but velocity skipped',
                throttle_duration_sec=2.0)

        self.publish(now)

    # ------------------------------------------------------------------
    def publish(self, now):
        stamp = now.to_msg()
        qx, qy, qz, qw = yaw_to_quat(self.theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        pc = [0.0] * 36
        pc[0] = pc[7] = max(self.var_xy, 1e-6)
        pc[14] = pc[21] = pc[28] = BIG
        pc[35] = max(self.var_yaw, 1e-6)
        odom.pose.covariance = pc

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        tc = [0.0] * 36
        tc[0] = self.var_vx
        tc[7] = 1e-3   # vy: nonholonomic, ~0 (small, not huge, so EKF can use it)
        tc[14] = tc[21] = tc[28] = BIG
        tc[35] = self.var_wz
        odom.twist.covariance = tc

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EncoderOdom()
    except ValueError as e:
        rclpy.logging.get_logger('encoder_odometry').fatal(str(e))
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()