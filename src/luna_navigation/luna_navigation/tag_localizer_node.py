import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from luna_sim.arena.layout import load_layout

from luna_interfaces.msg import TagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool


# math
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quaternion(yaw):
    """(x, y, z, w) for a rotation about z only"""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_matrix(x, y, z, w):
    """3x3 rotation matrix from a quaternion."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)],
    ])


def detection_in_base(p_opt, R_opt_tag, cov6_opt, R_base_opt, t_base_opt):
    """Tag detection in the camera optical frame -> (x_b, y_b, psi_b) in base_link + 3x3 covariance.

    psi_b = direction the tag's face (its z axis) points, seen from base_link.
    """
    p_b = R_base_opt @ p_opt + t_base_opt
    face_b = R_base_opt @ R_opt_tag[:, 2]
    psi_b = math.atan2(face_b[1], face_b[0])

    cov_pos_b = R_base_opt @ cov6_opt[:3, :3] @ R_base_opt.T
    yaw_var = max(cov6_opt[3, 3], cov6_opt[4, 4], cov6_opt[5, 5])   # conservative

    sigma_b = np.zeros((3, 3))
    sigma_b[:2, :2] = cov_pos_b[:2, :2]
    sigma_b[2, 2] = yaw_var
    return np.array([p_b[0], p_b[1], psi_b]), sigma_b


def robot_pose_from_tag(meas_b, sigma_b, tag_xy_yaw):
    """Measurement in base_link + known tag pose in map -> robot (x, y, theta) in map + 3x3 covariance."""
    xb, yb, psib = meas_b
    Xt, Yt, Psit = tag_xy_yaw

    th = wrap(Psit - psib)
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    dR = np.array([[-s, -c], [c, -s]])          # dR/dtheta
    m = np.array([xb, yb])

    xy = np.array([Xt, Yt]) - R @ m

    J = np.zeros((3, 3))                        # d(x, y, theta) / d(x_b, y_b, psi_b)
    J[:2, :2] = -R
    J[:2, 2] = dR @ m                           # yaw error on a far tag -> position error
    J[2, 2] = -1.0

    return np.array([xy[0], xy[1], th]), J @ sigma_b @ J.T


def apply_floor(cov, min_pos_std, min_yaw_std):
    cov = cov.copy()
    for i, f in ((0, min_pos_std), (1, min_pos_std), (2, min_yaw_std)):
        cov[i, i] = max(cov[i, i], f * f)
    return cov


def to_ros_cov(cov3, small=1e-6):
    """3x3 (x, y, yaw) -> 36-element row-major (x, y, z, roll, pitch, yaw)."""
    c = np.zeros((6, 6))
    idx = (0, 1, 5)
    for i, a in enumerate(idx):
        for j, b in enumerate(idx):
            c[a, b] = cov3[i, j]
    for k in (2, 3, 4):
        c[k, k] = small
    return c.flatten().tolist()


# ros
class TagLocalizerNode(Node):
    def __init__(self):
        super().__init__("tag_localizer_node")

        # Parameters
        self.declare_parameter('arena_json', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_use_range_m', 3.5)
        self.declare_parameter('init_max_range_m', 2.5)
        self.declare_parameter('min_position_std_m', 0.02)
        self.declare_parameter('min_yaw_std_rad', 0.02)

        def p(name):
            return self.get_parameter(name).value

        self.map_frame = p('map_frame')
        self.base_frame = p('base_frame')
        self.max_use_range = p('max_use_range_m')
        self.init_max_range = p('init_max_range_m')
        self.min_pos_std = p('min_position_std_m')
        self.min_yaw_std = p('min_yaw_std_rad')

        # Known tag poses: {id: (x, y, yaw)} in map
        tags = load_layout(p('arena_json'))['tags']
        self.tags = {int(t['id']): (float(t['x']), float(t['y']), float(t['yaw'])) for t in tags}
        self.get_logger().info(f'loaded {len(self.tags)} tags')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.extrinsics = {}

        # State
        self.initialized = False

        detection_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        init_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribers
        self.front_detection_sub = self.create_subscription(TagDetectionArray, "/cam_front/detections", self.extract_pose, detection_qos)
        self.rear_detection_sub = self.create_subscription(TagDetectionArray, "/cam_rear/detections", self.extract_pose, detection_qos)

        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/tag_pose", pose_qos)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/ekf_global/set_pose", 1)
        self.init_pub = self.create_publisher(Bool, "/localization/initialized", init_qos)

        self.init_pub.publish(Bool(data=False))

    def extract_pose(self, msg: TagDetectionArray):
        header = msg.header

        # base_link <- this camera's optical frame
        ext = self.get_extrinsic(header.frame_id)
        if ext is None:
            return
        R_bo, t_bo = ext

        for tag in msg.detections:
            known = self.tags.get(tag.id)
            if known is None:
                self.get_logger().debug(f'unknown tag id {tag.id}')
                continue

            pos = tag.pose.pose.position
            q = tag.pose.pose.orientation
            cov6 = np.array(tag.pose.covariance).reshape(6, 6)
            meas_b, sigma_b = detection_in_base(
                np.array([pos.x, pos.y, pos.z]),
                quat_to_matrix(q.x, q.y, q.z, q.w),
                cov6, R_bo, t_bo)

            rng = math.hypot(meas_b[0], meas_b[1])
            if rng > self.max_use_range:
                continue

            pose, cov = robot_pose_from_tag(meas_b, sigma_b, known)
            cov = apply_floor(cov, self.min_pos_std, self.min_yaw_std)

            self.publish_pose(pose, cov, header, rng, tag.id)

    def publish_pose(self, pose, cov, header, rng, tag_id):
        msg = PoseWithCovarianceStamped()

        msg.header.stamp = header.stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        qx, qy, qz, qw = yaw_to_quaternion(pose[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = to_ros_cov(cov)

        if self.initialized:
            self.pose_pub.publish(msg)
        elif rng <= self.init_max_range:         # only initialize from a close tag
            self.initialized = True
            self.init_pose_pub.publish(msg)
            self.init_pub.publish(Bool(data=True))
            self.get_logger().info(f'initialized from tag {tag_id} at {rng:.2f} m')

    def get_extrinsic(self, camera_frame):
        if camera_frame in self.extrinsics:
            return self.extrinsics[camera_frame]
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'no TF {self.base_frame} <- {camera_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return None
        r, t = tf.transform.rotation, tf.transform.translation
        ext = (quat_to_matrix(r.x, r.y, r.z, r.w), np.array([t.x, t.y, t.z]))
        self.extrinsics[camera_frame] = ext      # static, so cache it
        return ext


def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()