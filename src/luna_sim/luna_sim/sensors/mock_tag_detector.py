from visualization_msgs.msg import Marker, MarkerArray
from luna_sim.arena.layout import load_layout
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from luna_interfaces.msg import TagDetectionArray
from luna_interfaces.msg import TagDetection
import math
import numpy as np

def rotation_from_rpy(roll, pitch, yaw):
    """Convert roll, pitch, yaw angles into a 3D rotation matrix."""

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])

def transform_from_xyz_rpy(xyz, rpy):
    """Create a 4x4 transformation matrix from position and rotation."""

    T = np.eye(4)

    T[:3, :3] = rotation_from_rpy(*rpy)
    T[:3, 3] = np.array(xyz, dtype=float)

    return T

def inverse_transform(T):
    """Reverse a transformation matrix."""

    R = T[:3, :3]
    p = T[:3, 3]

    T_inv = np.eye(4)

    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ p

    return T_inv

def optical_to_camera_transform():
    """Express the camera optical frame in camera body coordinates."""

    return transform_from_xyz_rpy(
        xyz=[0.0, 0.0, 0.0],
        rpy=[-math.pi / 2, 0.0, -math.pi / 2]
    )

def camera_transform_in_map(robot_xyz, robot_rpy, camera_xyz, camera_rpy):
    """Calculate the camera optical frame's pose in the map frame."""

    T_map_base = transform_from_xyz_rpy(robot_xyz, robot_rpy)

    T_base_camera = transform_from_xyz_rpy(camera_xyz, camera_rpy)

    T_camera_optical = optical_to_camera_transform()

    return T_map_base @ T_base_camera @ T_camera_optical

def tag_position_in_optical(T_map_optical, tag_xyz):
    """Convert a tag's map position into camera optical coordinates."""

    tag_map = np.array([*tag_xyz, 1.0])

    tag_optical = inverse_transform(T_map_optical) @ tag_map

    return tag_optical[:3]

def tag_in_camera_view(tag_xyz, min_range, max_range, hfov_deg, vfov_deg):
    """Check if a tag is in front, within range, and inside the camera FOV."""

    x, y, z = tag_xyz

    # Tag must be in front of the camera
    if z <= 0:
        return False

    # Check distance
    distance = np.linalg.norm(tag_xyz)

    if distance < min_range or distance > max_range:
        return False

    # Calculate horizontal and vertical angles
    horizontal_angle = math.atan2(x, z)
    vertical_angle = math.atan2(y, z)

    # Convert FOV limits from degrees to radians
    hfov_limit = math.radians(hfov_deg / 2)
    vfov_limit = math.radians(vfov_deg / 2)

    return (
        abs(horizontal_angle) <= hfov_limit
        and abs(vertical_angle) <= vfov_limit
    )

def projected_tag_pixels(tag_size_m, depth_m, view_angle_rad,
                         image_width_px, hfov_deg):
    """Estimate the apparent horizontal width of an AprilTag in pixels."""

    if depth_m <= 0:
        return 0.0

    # Calculate focal length in pixels
    hfov_rad = math.radians(hfov_deg)
    fx = image_width_px / (2.0 * math.tan(hfov_rad / 2.0))

    # Reduce apparent width when viewed at an angle
    apparent_width = tag_size_m * max(0.0, math.cos(view_angle_rad))

    # Calculate projected width
    return fx * apparent_width / depth_m

def tag_view_angle(tag_xyz, tag_yaw, camera_xyz):
    """Calculate the viewing angle between a tag's front and the camera."""

    # Direction the tag is facing in the arena
    tag_normal = np.array([
        math.cos(tag_yaw),
        math.sin(tag_yaw),
        0.0
    ])

    # Direction from the tag to the camera
    to_camera = np.array(camera_xyz) - np.array(tag_xyz)

    distance = np.linalg.norm(to_camera)

    if distance < 1e-9:
        return math.pi

    to_camera = to_camera / distance

    # Angle between the tag normal and camera direction
    cosine = np.clip(np.dot(tag_normal, to_camera), -1.0, 1.0)

    return math.acos(cosine)

def is_tag_detectable(T_map_optical, tag, config):
    """Determine whether a camera can detect an AprilTag."""

    tag_xyz = [tag['x'], tag['y'], tag['z']]

    # Convert the tag position into the camera optical frame
    tag_optical = tag_position_in_optical(T_map_optical, tag_xyz)

    # Check distance and field of view
    if not tag_in_camera_view(
        tag_optical,
        config['min_range_m'],
        config['max_range_m'],
        config['hfov_deg'],
        config['vfov_deg']
    ):
        return False

    # Check whether the tag faces the camera
    camera_xyz = T_map_optical[:3, 3]

    angle = tag_view_angle(tag_xyz, tag['yaw'], camera_xyz)

    if angle > math.radians(config['max_view_angle_deg']):
        return False

    # Check whether the tag is large enough in the image
    pixels = projected_tag_pixels(
        tag['size'],
        tag_optical[2],
        angle,
        config['image_width_px'],
        config['hfov_deg']
    )

    return pixels >= config['min_tag_pixels']

def tag_pose_in_optical(T_map_optical, tag):
    """Calculate the tag's complete pose in the camera optical frame."""

    yaw = tag['yaw']

    # Tag's coordinate axes expressed in the map frame
    tag_x = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    tag_y = np.array([0.0, 0.0, 1.0])
    tag_z = np.array([math.cos(yaw), math.sin(yaw), 0.0])

    # Build the tag's transformation in the map frame
    T_map_tag = np.eye(4)

    T_map_tag[:3, :3] = np.column_stack((tag_x, tag_y, tag_z))
    T_map_tag[:3, 3] = [tag['x'], tag['y'], tag['z']]

    # Convert from map coordinates to camera optical coordinates
    return inverse_transform(T_map_optical) @ T_map_tag

def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to [x, y, z, w]."""

    trace = np.trace(R)

    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s

    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s

    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w])

    return q / np.linalg.norm(q)

def add_detection_noise(T_optical_tag, view_angle, config, rng):
    """Add Gaussian position/rotation noise and calculate covariance."""

    distance = np.linalg.norm(T_optical_tag[:3, 3])

    # Calculate noise standard deviations
    pos_std = (
        config['position_noise_base_m']
        + config['position_noise_per_m2'] * distance**2
    )

    rot_std = (
        config['rotation_noise_base_rad']
        + config['rotation_noise_per_rad'] * view_angle
    )

    # Add random noise to position
    T_noisy = T_optical_tag.copy()
    T_noisy[:3, 3] += rng.normal(0.0, pos_std, 3)

    # Add small random rotations
    rotation_error = rng.normal(0.0, rot_std, 3)

    T_noisy[:3, :3] = (
        T_optical_tag[:3, :3]
        @ rotation_from_rpy(*rotation_error)
    )

    # Covariance: x, y, z, roll, pitch, yaw
    covariance = np.diag(
        [pos_std**2] * 3 + [rot_std**2] * 3
    )

    return T_noisy, covariance

def make_tag_detection(tag, T_noisy, covariance):
    """Convert a simulated tag measurement into a ROS 2 message."""

    detection = TagDetection()

    # Tag identifier
    detection.id = int(tag['id'])

    # Position in camera optical coordinates
    detection.pose.pose.position.x = float(T_noisy[0, 3])
    detection.pose.pose.position.y = float(T_noisy[1, 3])
    detection.pose.pose.position.z = float(T_noisy[2, 3])

    # Convert rotation matrix to quaternion
    q = rotation_matrix_to_quaternion(T_noisy[:3, :3])

    detection.pose.pose.orientation.x = float(q[0])
    detection.pose.pose.orientation.y = float(q[1])
    detection.pose.pose.orientation.z = float(q[2])
    detection.pose.pose.orientation.w = float(q[3])

    # ROS expects 36 covariance values in row-major order
    detection.pose.covariance = covariance.flatten().tolist()

    return detection

def quaternion_to_rpy(q):
    """Convert a ROS quaternion into roll, pitch, yaw."""

    x, y, z, w = q.x, q.y, q.z, q.w

    # Roll (rotation around X)
    roll = math.atan2(
        2.0 * (w*x + y*z),
        1.0 - 2.0 * (x*x + y*y)
    )

    # Pitch (rotation around Y)
    sin_pitch = 2.0 * (w*y - z*x)
    pitch = math.asin(np.clip(sin_pitch, -1.0, 1.0))

    # Yaw (rotation around Z)
    yaw = math.atan2(
        2.0 * (w*z + x*y),
        1.0 - 2.0 * (y*y + z*z)
    )

    return [roll, pitch, yaw]

def load_camera_geometry(geometry_yaml, cameras):
    """Load camera mounting positions and orientations from YAML."""

    with open(geometry_yaml, 'r') as file:
        geometry = yaml.safe_load(file)

    sensors = geometry['sensors']
    camera_poses = {}

    for camera in cameras:
        if camera not in sensors:
            raise ValueError(f"Camera '{camera}' not found in geometry YAML")

        camera_poses[camera] = {
            'xyz': sensors[camera]['xyz'],
            'rpy': sensors[camera]['rpy']
        }

    return camera_poses

class MockTagDetector(Node):
    """ROS 2 node that simulates AprilTag detections."""

    def __init__(self):
        super().__init__('mock_tag_detector')

        # Declare parameters
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('cameras', ['cam_front', 'cam_rear'])

        # Load camera mounting geometry
        self.declare_parameter('geometry_yaml', '')

        geometry_path = self.get_parameter('geometry_yaml').value

        if not geometry_path:
            raise ValueError("geometry_yaml parameter is required")

        self.camera_poses = load_camera_geometry(
            geometry_path,
            self.get_parameter('cameras').value
        )

        # Create one publisher for each camera
        self.detection_publishers = {}

        for camera in self.get_parameter('cameras').value:
            topic = f'/{camera}/detections'

            self.detection_publishers[camera] = self.create_publisher(
                TagDetectionArray,
                topic,
                10
            )

        # Publish visualization markers for RViz
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/tags/markers',
            10
        )

        self.get_logger().info('Mock AprilTag detector initialized')

	# Store the rover's latest ground-truth pose
        self.latest_ground_truth = None

        # Subscribe to the simulated rover position
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/sim/ground_truth/pose',
            self.ground_truth_callback,
            10
        )
        # Get the generated arena JSON path
        self.declare_parameter('arena_json', '')

        arena_path = self.get_parameter('arena_json').value

        if not arena_path:
            raise ValueError("arena_json parameter is required")

        # Load AprilTag positions from the arena
        layout = load_layout(arena_path)

        self.tags = layout['tags']

        self.get_logger().info(
            f"Loaded {len(self.tags)} AprilTags from arena"
        )

        # Detection settings from ROS parameters
        defaults = {
            'position_noise_base_m': 0.005,
            'position_noise_per_m2': 0.004,
            'rotation_noise_base_rad': 0.01,
            'rotation_noise_per_rad': 0.05,
            'min_range_m': 0.2,
            'max_range_m': 4.0,
            'hfov_deg': 90.0,
            'vfov_deg': 60.0,
            'max_view_angle_deg': 70.0,
            'image_width_px': 1280,
            'min_tag_pixels': 20.0
        }

        self.config = {}

        for name, default in defaults.items():
            self.config[name] = self.declare_parameter(
                name, default
            ).value

        # Initialize reproducible random noise
        self.declare_parameter('seed', 0)

        seed = self.get_parameter('seed').value
        self.rng = np.random.default_rng(seed)

        # Publish detections at the configured rate
        rate_hz = float(self.get_parameter('rate_hz').value)

        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")

        self.publish_timer = self.create_timer(
            1.0 / rate_hz,
            self.publish_detections
        )

    def ground_truth_callback(self, msg):
        """Store the latest rover pose received from the simulator."""

        self.latest_ground_truth = msg

    def get_robot_pose(self):
        """Return the rover's latest XYZ position and RPY orientation."""

        if self.latest_ground_truth is None:
            return None

        pose = self.latest_ground_truth.pose

        xyz = [
            pose.position.x,
            pose.position.y,
            pose.position.z
        ]

        rpy = quaternion_to_rpy(pose.orientation)

        return xyz, rpy

    def get_camera_transforms(self):
        """Calculate camera positions and orientations in the arena."""

        robot_pose = self.get_robot_pose()

        if robot_pose is None:
            return None

        robot_xyz, robot_rpy = robot_pose

        transforms = {}

        for camera, pose in self.camera_poses.items():
            transforms[camera] = camera_transform_in_map(
                robot_xyz,
                robot_rpy,
                pose['xyz'],
                pose['rpy']
            )

        return transforms

    def get_visible_tags(self):
        """Find which AprilTags are visible to each camera."""

        transforms = self.get_camera_transforms()

        if transforms is None:
            return None

        visible_tags = {}

        for camera, T_map_optical in transforms.items():

            visible_tags[camera] = []

            for tag in self.tags:

                if is_tag_detectable(
                    T_map_optical,
                    tag,
                    self.config
                ):
                    visible_tags[camera].append(tag)

        return visible_tags

    def make_noisy_detection(self, camera, tag):
        """Create a noisy detection for a visible AprilTag."""

        # Get the camera's position and orientation
        T_map_optical = self.get_camera_transforms()[camera]

        # Calculate the tag's complete pose
        T_optical_tag = tag_pose_in_optical(T_map_optical, tag)

        # Calculate the viewing angle
        tag_xyz = [tag['x'], tag['y'], tag['z']]
        camera_xyz = T_map_optical[:3, 3]

        angle = tag_view_angle(tag_xyz, tag['yaw'], camera_xyz)

        # Add realistic measurement errors
        T_noisy, covariance = add_detection_noise(
            T_optical_tag,
            angle,
            self.config,
            self.rng
        )

        # Convert into the custom ROS 2 message
        return make_tag_detection(tag, T_noisy, covariance)

    def make_detection_arrays(self):
        """Create detection arrays for both cameras."""

        visible = self.get_visible_tags()

        if visible is None:
            return None

        detection_arrays = {}

        for camera, tags in visible.items():

            msg = TagDetectionArray()

            # Camera optical frame and capture timestamp
            msg.header.frame_id = f'{camera}_optical'
            msg.header.stamp = self.latest_ground_truth.header.stamp

            # Add detected tags (empty if none visible)
            for tag in tags:
                detection = self.make_noisy_detection(camera, tag)
                msg.detections.append(detection)

            detection_arrays[camera] = msg

        return detection_arrays

    def publish_detections(self):
        """Publish both camera arrays and static/dynamic RViz markers at 10 Hz."""
        detection_arrays = self.make_detection_arrays()
        if detection_arrays is None:
            return

        for camera, msg in detection_arrays.items():
            self.detection_publishers[camera].publish(msg)

        marker_array = self.make_tag_markers()
        marker_array.markers.extend(self.make_sightline_markers().markers)
        self.marker_publisher.publish(marker_array)


    def make_tag_markers(self):
        """Create RViz markers for all AprilTags in the arena."""

        markers = MarkerArray()

        for tag in self.tags:
            marker = Marker()

            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = 'april_tags'
            marker.id = int(tag['id'])
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position in the arena
            marker.pose.position.x = float(tag['x'])
            marker.pose.position.y = float(tag['y'])
            marker.pose.position.z = float(tag['z'])

            # Orient the square so its front faces the tag's yaw direction
            R = rotation_from_rpy(
                math.pi / 2,
                0.0,
                tag['yaw'] + math.pi / 2
            )

            q = rotation_matrix_to_quaternion(R)

            marker.pose.orientation.x = float(q[0])
            marker.pose.orientation.y = float(q[1])
            marker.pose.orientation.z = float(q[2])
            marker.pose.orientation.w = float(q[3])

            # Physical dimensions of the AprilTag
            marker.scale.x = float(tag['size'])
            marker.scale.y = float(tag['size'])
            marker.scale.z = 0.01

            # Yellow markers for visibility
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.1
            marker.color.a = 1.0

            markers.markers.append(marker)

        return markers

    def make_sightline_markers(self):
        """Draw map-frame line segments only for currently detected tags.

        A stable marker ID is assigned to each camera. DELETE clears an old
        sightline immediately if a camera loses all its visible AprilTags.
        """
        result = MarkerArray()
        transforms = self.get_camera_transforms()
        visible = self.get_visible_tags()
        if transforms is None or visible is None:
            return result

        for marker_id, camera in enumerate(self.detection_publishers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.latest_ground_truth.header.stamp
            marker.ns = 'april_tag_sightlines'
            marker.id = marker_id
            marker.type = Marker.LINE_LIST
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.018
            marker.color.a = 1.0
            if camera == 'cam_front':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.2
                marker.color.b = 1.0

            tags = visible.get(camera, [])
            marker.action = Marker.ADD if tags else Marker.DELETE
            camera_xyz = transforms[camera][:3, 3]
            for tag in tags:
                start = Point(
                    x=float(camera_xyz[0]),
                    y=float(camera_xyz[1]),
                    z=float(camera_xyz[2]),
                )
                end = Point(
                    x=float(tag['x']),
                    y=float(tag['y']),
                    z=float(tag['z']),
                )
                marker.points.extend((start, end))
            result.markers.append(marker)
        return result

def main(args=None):
    """Start and run the mock AprilTag detector node."""

    rclpy.init(args=args)

    node = MockTagDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
