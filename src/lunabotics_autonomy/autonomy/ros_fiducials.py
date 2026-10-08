"""Optional ArUco detector. Inhibited until tag map and camera extrinsic calibrated."""
import json
import math
from .fiducials import tag_object_corners, valid_transform, world_base_pose
from .orbbec import camera_topics


def main(args=None):
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from geometry_msgs.msg import PoseWithCovarianceStamped

    class FiducialNode(Node):
        def __init__(self):
            super().__init__('lunabotics_fiducials')
            self.declare_parameter('marker_size_m', 0.0)
            self.declare_parameter('tag_map_json', '{}')
            self.declare_parameter('base_camera_transform_json', '[]')
            self.declare_parameter('camera_name', 'camera')
            self.declare_parameter('camera_frame', '')
            self.declare_parameter('max_reprojection_px', 3.0)
            self.declare_parameter('max_pair_disagreement_m', 0.25)
            self.declare_parameter('max_pair_disagreement_rad', 0.3)
            self.marker_size = float(self.get_parameter('marker_size_m').value)
            topics = camera_topics(self.get_parameter('camera_name').value)
            self.camera_frame = (self.get_parameter('camera_frame').value or
                                 topics.color_frame)
            self.tag_map = json.loads(self.get_parameter('tag_map_json').value)
            extrinsic = json.loads(self.get_parameter('base_camera_transform_json').value)
            self.base_camera = np.asarray(extrinsic, dtype=float) if extrinsic else None
            self.enabled = (self.marker_size > 0 and bool(self.tag_map) and
                            self.base_camera is not None)
            if self.enabled:
                try:
                    tag_object_corners(self.marker_size)
                    valid_transform(self.base_camera)
                    for tf in self.tag_map.values():
                        valid_transform(tf)
                except (ValueError, TypeError) as exc:
                    self.enabled = False
                    self.get_logger().error(f'Invalid camera calibration: {exc}')
            if not self.enabled:
                self.get_logger().warn('Camera detector inhibited: calibrate tags, size and base-camera extrinsic')
            self.info = None
            self.pub = self.create_publisher(PoseWithCovarianceStamped, '/camera/pose', 10)
            self.create_subscription(CameraInfo, topics.color_info, self.on_info,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, topics.color_image, self.on_image,
                                     qos_profile_sensor_data)
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.detector = (cv2.aruco.ArucoDetector(dictionary)
                             if hasattr(cv2.aruco, 'ArucoDetector') else None)
            self.dictionary = dictionary

        def on_info(self, msg):
            if msg.header.frame_id == self.camera_frame:
                self.info = msg

        def on_image(self, msg):
            if not self.enabled or self.info is None:
                return
            if msg.header.frame_id != self.camera_frame:
                return
            if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
                return
            if (self.info.width != msg.width or self.info.height != msg.height):
                return
            channels = 1 if msg.encoding == 'mono8' else 3
            if msg.step < msg.width * channels or len(msg.data) < msg.step * msg.height:
                return
            image = np.ndarray((msg.height, msg.step), dtype=np.uint8, buffer=bytes(msg.data))
            image = image[:, :msg.width*channels]
            if channels == 3:
                image = image.reshape((msg.height, msg.width, 3))
                image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8'
                                    else cv2.COLOR_BGR2GRAY)
            if self.detector:
                corners, ids, _ = self.detector.detectMarkers(image)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(image, self.dictionary)
            if ids is None:
                return
            camera_matrix = np.asarray(self.info.k, dtype=np.float64).reshape((3, 3))
            distortion = np.asarray(self.info.d, dtype=np.float64)
            objects = tag_object_corners(self.marker_size)
            estimates = []
            for polygon, marker_id in zip(corners, ids.flatten()):
                world_tag = self.tag_map.get(str(int(marker_id)))
                if world_tag is None:
                    continue
                ok, rvec, tvec = cv2.solvePnP(objects, polygon.reshape(4, 2),
                                               camera_matrix, distortion,
                                               flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok or tvec[2, 0] <= 0:
                    continue
                predicted, _ = cv2.projectPoints(objects, rvec, tvec,
                                                   camera_matrix, distortion)
                error = float(np.sqrt(np.mean((predicted.reshape(4, 2) -
                                               polygon.reshape(4, 2))**2)))
                if error > float(self.get_parameter('max_reprojection_px').value):
                    continue
                camera_tag = np.eye(4)
                camera_tag[:3, :3] = cv2.Rodrigues(rvec)[0]
                camera_tag[:3, 3] = tvec.flatten()
                try:
                    estimates.append((error, world_base_pose(world_tag, camera_tag,
                                                              self.base_camera)))
                except ValueError:
                    continue
            if not estimates:
                return
            estimates.sort(key=lambda x: x[0])
            best = estimates[0][1]
            # Reject a frame with disagreeing visible tags, rather than cherry-pick a pose.
            for _, pose in estimates[1:]:
                dyaw = (pose[2] - best[2] + math.pi) % (2*math.pi) - math.pi
                if (math.hypot(pose[0]-best[0], pose[1]-best[1]) >
                    float(self.get_parameter('max_pair_disagreement_m').value) or
                    abs(dyaw) > float(self.get_parameter('max_pair_disagreement_rad').value)):
                    return
            out = PoseWithCovarianceStamped()
            out.header.stamp, out.header.frame_id = msg.header.stamp, 'map'
            out.pose.pose.position.x, out.pose.pose.position.y = best[:2]
            out.pose.pose.orientation.z = math.sin(best[2]/2)
            out.pose.pose.orientation.w = math.cos(best[2]/2)
            # Conservative nominal covariance; calibrate against surveying, not just pixels.
            out.pose.covariance[0] = out.pose.covariance[7] = 0.2**2
            out.pose.covariance[35] = 0.2**2
            self.pub.publish(out)

    rclpy.init(args=args)
    node = FiducialNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
