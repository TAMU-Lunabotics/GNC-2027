"""
Publishes /clock so the whole graph runs on simulated time, faster than real time.

This node must run with use_sim_time:=false (it is the time source).
Every other node runs with use_sim_time:=true.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock


class SimClock(Node):
    def __init__(self) -> None:
        super().__init__("sim_clock")
        if self.get_parameter("use_sim_time").value:
            raise RuntimeError("sim_clock is the time source; launch it with use_sim_time:=false")

        self.rtf = float(self.declare_parameter("real_time_factor", 20.0).value)
        rate_hz = float(self.declare_parameter("publish_rate_hz", 200.0).value)
        self.sim_t = float(self.declare_parameter("start_time_s", 0.0).value)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Clock, "/clock", qos)
        self.last_wall = time.monotonic()
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"publishing /clock at {rate_hz:.0f} Hz wall, {self.rtf:.1f}x real time, (sim step ~{self.rtf / rate_hz * 1000:.1f} ms)")

    def _tick(self) -> None:
        now = time.monotonic()
        self.sim_t += (now - self.last_wall) * self.rtf
        self.last_wall = now
        msg = Clock()
        msg.clock.sec = int(self.sim_t)
        msg.clock.nanosec = int((self.sim_t - int(self.sim_t)) * 1e9)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()