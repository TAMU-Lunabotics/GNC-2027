"""GNC wheel-target conversion. NOT a physical RoboClaw motor driver.

This node publishes /auto_drive QPPS targets for an explicitly validated adapter.
Do not bridge it to physical hardware until wheel dimensions, encoder scale,
actuation sign, command timeout and independent stop path are verified.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


def wheel_targets_qpps(
    linear_mps: float,
    angular_radps: float,
    track_width_m: float,
    wheel_radius_m: float,
    ticks_per_wheel_rev: float,
    max_wheel_speed_mps: float,
) -> tuple[int, int]:
    """Convert a planar twist to left/right QPPS, preserving steering ratio.

    There is intentionally no minimum nonzero speed: zero must stay zero,
    reverse must stay negative, and small commands must not jump forward.
    """
    values = (
        linear_mps, angular_radps, track_width_m, wheel_radius_m,
        ticks_per_wheel_rev, max_wheel_speed_mps,
    )
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError("nonfinite/non-numeric motion or wheel calibration")
    if min(track_width_m, wheel_radius_m, ticks_per_wheel_rev, max_wheel_speed_mps) <= 0:
        raise ValueError("measured positive wheel calibration and speed limit required")

    left_mps = linear_mps - angular_radps * track_width_m / 2.0
    right_mps = linear_mps + angular_radps * track_width_m / 2.0
    scale = max(1.0, abs(left_mps) / max_wheel_speed_mps,
                abs(right_mps) / max_wheel_speed_mps)
    conversion = ticks_per_wheel_rev / (2.0 * math.pi * wheel_radius_m)
    return (round(left_mps / scale * conversion),
            round(right_mps / scale * conversion))


class DriveControllerNode(Node):
    def __init__(self):
        super().__init__("drive_controller")
        p = self.declare_parameter
        # Zero defaults deliberately prevent uncalibrated operation.
        p("track_width_m", 0.0)
        p("wheel_radius_m", 0.0)
        p("ticks_per_wheel_rev", 0.0)
        p("max_wheel_speed_mps", 0.0)
        get = lambda key: float(self.get_parameter(key).value)
        self.geometry = (
            get("track_width_m"),
            get("wheel_radius_m"),
            get("ticks_per_wheel_rev"),
            get("max_wheel_speed_mps"),
        )
        wheel_targets_qpps(0.0, 0.0, *self.geometry)
        self.drive_pub = self.create_publisher(Int32MultiArray, "/auto_drive", 10)
        self.cmd_sub = self.create_subscription(
            TwistStamped, "/cmd_vel", self.control_loop, 10
        )
        self.get_logger().warn(
            "GNC /auto_drive QPPS output only; no physical actuator bridge is installed"
        )

    def control_loop(self, msg):
        try:
            targets = wheel_targets_qpps(
                msg.twist.linear.x, msg.twist.angular.z, *self.geometry
            )
        except ValueError as exc:
            targets = (0, 0)
            self.get_logger().error(f"Rejected motion command: {exc}")
        self.drive_pub.publish(Int32MultiArray(data=list(targets)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DriveControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
