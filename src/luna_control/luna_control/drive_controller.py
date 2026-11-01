import rclpy
from rclpy import Node
import math

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import TwistStamped

WHEEL_RADIUS = 0.048
TICKS_PER_REV = 28 * 19.2 # 28 ppr encoder, 19.2:1 gearbox in the motor, may be missing extra external reduction
MAX_SPEED = 0.1 # m/s
MIN_SPEED = 0.25 # m/s

class DriveController(Node):
    def __init__(self):
        super.__init__("drive_controller")

        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        self.track_width = float(self.declare_parameter('track_width', 0.8).value)
        # state
        self.setpoints = [(0,0), (0,0)]
        self.odom = None

        # Subscirbers
        # self.odom_sub = self.create_subscription(Odometry, "/wheel/odometry", 10, self.odom_cb)
        self.cmd_sub = self.create_subscription(TwistStamped, "/cmd_vel", 10, self.control_loop)

        # Publishers
        self.drive_pub = self.create_publisher(Int32MultiArray, "/auto_drive", 10)

        self.create_timer(self.rate_hz, self.control_loop)


    def mps_to_qpps(v):
        return int(v / (2 * math.pi * WHEEL_RADIUS) * TICKS_PER_REV)

    def control_loop(self, msg):
        v_left = self.mps_to_qpps(msg.twist.linear.x - msg.twist.angular.z*self.track_width / 2)
        v_right = self.mps_to_qpps(msg.twist.linear.x + msg.twist.angular.z*self.track_width / 2)

        v_left = max(MIN_SPEED, min(self.mps_to_qpps(MIN_SPEED), self.mps_to_qpps(MAX_SPEED, v_left)))
        v_right = max(MIN_SPEED, min(self.mps_to_qpps(MIN_SPEED), self.mps_to_qpps(MAX_SPEED, v_right)))

        vel_msg = Int32MultiArray()
        vel_msg.data = [v_left, v_right]

        self.get_logger().info(f'QPPS: left: {v_left}, right: {v_right}')

        self.drive_pub.publish(vel_msg)