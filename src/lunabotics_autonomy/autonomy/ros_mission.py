"""ROS 2 adapter. No motor/actuator driver is included or enabled."""
import math
from collections import deque
from .mission import Mission
from .occupancy import ObstacleMemory
from .orbbec import camera_topics
from .types import Config, Observation, Pose


def yaw_from_quaternion(q):
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        return float('nan')
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def _stamp(msg):
    return msg.sec + msg.nanosec * 1e-9


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.duration import Duration
    from rclpy.time import Time
    from std_srvs.srv import Trigger
    from std_msgs.msg import Bool, Float32, Int32MultiArray, String
    from nav_msgs.msg import Odometry, OccupancyGrid, Path
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.msg import CameraInfo, Image
    from sensor_msgs_py import point_cloud2
    from tf2_ros import Buffer, TransformListener
    from .depth import depth_points

    class MissionNode(Node):
        def __init__(self):
            super().__init__('lunabotics_mission')
            self.declare_parameter('armed', False)
            self.declare_parameter('geometry_confirmed', False)
            self.declare_parameter('hopper_limit_l', 0.0)
            self.declare_parameter('bulk_density_kg_m3', 0.0)
            self.declare_parameter('target_l', 25.0)
            self.declare_parameter('per_cycle_l', 14.0)
            self.declare_parameter('dig_x', 6.9)
            self.declare_parameter('dig_y', 2.2)
            self.declare_parameter('dump_x', 1.1)
            self.declare_parameter('dump_y', 2.2)
            self.declare_parameter('dig_yaw', 0.0)
            self.declare_parameter('dump_yaw', math.pi)
            self.declare_parameter('arena_width', 7.9)
            self.declare_parameter('arena_height', 4.4)
            self.declare_parameter('robot_radius', 0.48)
            self.declare_parameter('minimum_battery_v', 20.0)
            self.declare_parameter('lidar_min_z', -0.15)
            self.declare_parameter('lidar_max_z', 0.65)
            self.declare_parameter('lidar_max_range', 8.0)
            self.declare_parameter('require_depth', False)
            self.declare_parameter('camera_name', 'camera')
            p = lambda name: self.get_parameter(name).value
            topics = camera_topics(p('camera_name'))
            self.lidar_min_z = float(p('lidar_min_z'))
            self.lidar_max_z = float(p('lidar_max_z'))
            self.lidar_max_range = float(p('lidar_max_range'))
            self.last_cloud_attempt = -1e9
            self.last_depth_attempt = -1e9
            self.last_visualization = -1e9
            self.lidar_points = []
            self.depth_obstacles = []
            self.depth_info = None
            self.armed = bool(p('armed')) and bool(p('geometry_confirmed'))
            self.obs = Observation(now=0.0, estop=True)
            self.pose_history = deque(maxlen=100)
            self.mission = None
            self.obstacle_memory = None
            try:
                cfg = Config(width=float(p('arena_width')), height=float(p('arena_height')),
                             robot_radius=float(p('robot_radius')),
                             dig=(float(p('dig_x')), float(p('dig_y'))),
                             dump=(float(p('dump_x')), float(p('dump_y'))),
                             dig_yaw=float(p('dig_yaw')),
                             dump_yaw=float(p('dump_yaw')),
                             hopper_limit_l=float(p('hopper_limit_l')),
                             bulk_density_kg_m3=float(p('bulk_density_kg_m3')),
                             target_l=float(p('target_l')),
                             per_cycle_l=float(p('per_cycle_l')),
                             min_battery_v=float(p('minimum_battery_v')),
                             require_depth=bool(p('require_depth')))
                self.mission = Mission(cfg)
                self.obstacle_memory = ObstacleMemory(cfg)
            except ValueError as exc:
                self.get_logger().error(f'Inhibited: configuration invalid: {exc}')
            if not self.armed:
                self.get_logger().warn('Disarmed: set armed and geometry_confirmed only after verification')
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            self.drive = self.create_publisher(Twist, '/autonomy/cmd_vel', 10)
            self.tool = self.create_publisher(String, '/autonomy/tool_command', 10)
            self.state = self.create_publisher(String, '/autonomy/state', 10)
            self.grid_pub = self.create_publisher(OccupancyGrid, '/autonomy/obstacle_grid', 10)
            self.path_pub = self.create_publisher(Path, '/autonomy/planned_path', 10)
            self.create_subscription(Odometry, '/odometry/filtered', self.odom, 10)
            self.create_subscription(PoseWithCovarianceStamped, '/camera/pose', self.camera, 10)
            self.create_subscription(PointCloud2, '/unilidar/cloud', self.cloud, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, topics.depth_info, self.depth_calibration,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, topics.depth_image, self.depth,
                                     qos_profile_sensor_data)
            self.create_subscription(Int32MultiArray, '/drive/encoders', self.encoders, 10)
            self.create_subscription(Float32, '/hopper/mass_kg', self.mass, 10)
            self.create_subscription(Float32, '/battery/voltage', self.battery, 10)
            self.create_subscription(String, '/actuator/status', self.actuator, 10)
            self.create_subscription(Bool, '/berm/baseline_ready', self.berm_baseline, 10)
            self.create_subscription(Float32, '/berm/volume_l', self.berm_volume, 10)
            self.create_subscription(Bool, '/safety/estop', self.estop, 10)
            self.create_service(Trigger, '/autonomy/start', self.start)
            self.create_service(Trigger, '/autonomy/reset', self.reset)
            self.create_timer(0.05, self.step)

        def clock(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def odom(self, msg):
            if msg.header.frame_id != 'map' or msg.child_frame_id != 'base_link':
                return  # A mismatched TF tree must not produce a plausible position.
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.obs.pose = Pose(p.x, p.y, yaw_from_quaternion(q))
            self.obs.pose_time = _stamp(msg.header.stamp)
            self.pose_history.append((self.obs.pose_time,self.obs.pose))
            covariance = msg.pose.covariance
            variances = (covariance[0], covariance[7], covariance[35])
            self.obs.pose_sigma = (math.sqrt(max(variances)) if
                all(math.isfinite(v) and v >= 0 for v in variances) else float('inf'))

        def pose_at(self, stamp):
            if not self.pose_history:
                return None
            time,pose = min(self.pose_history,key=lambda item:abs(item[0]-stamp))
            return pose if abs(time-stamp) <= .1 else None

        def camera(self, msg):
            if msg.header.frame_id == 'map':
                self.obs.camera_time = _stamp(msg.header.stamp)

        def cloud(self, msg):
            if self.clock() - self.last_cloud_attempt < 0.15:
                return
            self.last_cloud_attempt = self.clock()
            try:
                transform = self.tf.lookup_transform('base_link', msg.header.frame_id,
                       Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.02))
                q, offset = transform.transform.rotation, transform.transform.translation
                # Rotate sensor points into base_link, then into map using fused pose.
                yaw = yaw_from_quaternion(q)
                # A nonplanar LiDAR requires the full quaternion rotation.
                xx, yy, zz, ww = q.x, q.y, q.z, q.w
                n = math.sqrt(xx*xx+yy*yy+zz*zz+ww*ww)
                xx, yy, zz, ww = xx/n, yy/n, zz/n, ww/n
                pose = self.pose_at(_stamp(msg.header.stamp))
                if not math.isfinite(yaw) or pose is None:
                    return
                c, s = math.cos(pose.yaw), math.sin(pose.yaw)
                points = []
                count = 0
                for x, y, z in point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
                    count += 1
                    # q * vector * conjugate(q), expanded without dependencies.
                    tx, ty, tz = 2*(yy*z-zz*y), 2*(zz*x-xx*z), 2*(xx*y-yy*x)
                    bx = x + ww*tx + (yy*tz-zz*ty) + offset.x
                    by = y + ww*ty + (zz*tx-xx*tz) + offset.y
                    bz = z + ww*tz + (xx*ty-yy*tx) + offset.z
                    if (self.lidar_min_z <= bz <= self.lidar_max_z and
                        0.08 < math.hypot(bx, by) <= self.lidar_max_range):
                        points.append((pose.x + c*bx - s*by, pose.y + s*bx + c*by))
                if count < 10:
                    return
                sampled = points[::max(1, len(points)//1500)]
                self.obstacle_memory.update((pose.x, pose.y), sampled,
                                            _stamp(msg.header.stamp))
                self.lidar_points = self.obstacle_memory.points(self.clock())
                self.obs.lidar_time = _stamp(msg.header.stamp)
            except (Exception,) as exc:
                self.get_logger().warn(f'LiDAR/TF rejection: {exc}', throttle_duration_sec=3.0)

        def depth_calibration(self, msg):
            self.depth_info = msg

        def depth(self, msg):
            if not self.mission or not self.mission.cfg.require_depth:
                return
            if self.clock() - self.last_depth_attempt < 0.15:
                return
            self.last_depth_attempt = self.clock()
            if (self.depth_info is None or
                self.depth_info.header.frame_id != msg.header.frame_id or
                (self.depth_info.width,self.depth_info.height) != (msg.width,msg.height)):
                return
            try:
                tf = self.tf.lookup_transform('base_link', msg.header.frame_id,
                     Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.02))
                q, shift = tf.transform.rotation, tf.transform.translation
                xx, yy, zz, ww = q.x, q.y, q.z, q.w
                norm = math.sqrt(xx*xx+yy*yy+zz*zz+ww*ww)
                xx, yy, zz, ww = xx/norm, yy/norm, zz/norm, ww/norm
                pose = self.pose_at(_stamp(msg.header.stamp))
                if pose is None:
                    return
                c, s = math.cos(pose.yaw), math.sin(pose.yaw)
                hits = []
                for x, y, z in depth_points(msg.data, msg.width, msg.height,
                                              msg.step, msg.encoding, msg.is_bigendian,
                                              self.depth_info.k):
                    tx, ty, tz = 2*(yy*z-zz*y), 2*(zz*x-xx*z), 2*(xx*y-yy*x)
                    bx = x + ww*tx + (yy*tz-zz*ty) + shift.x
                    by = y + ww*ty + (zz*tx-xx*tz) + shift.y
                    bz = z + ww*tz + (xx*ty-yy*tx) + shift.z
                    if self.lidar_min_z <= bz <= self.lidar_max_z:
                        hits.append((pose.x+c*bx-s*by, pose.y+s*bx+c*by))
                self.depth_obstacles = hits
                self.obs.depth_time = _stamp(msg.header.stamp)
            except Exception as exc:
                self.get_logger().warn(f'Depth/TF rejection: {exc}', throttle_duration_sec=3.0)

        def encoders(self, msg):
            if len(msg.data) >= 2:
                self.obs.encoder_time = self.clock()

        def mass(self, msg):
            self.obs.mass_kg, self.obs.mass_time = float(msg.data), self.clock()

        def battery(self, msg):
            self.obs.battery_v, self.obs.battery_time = float(msg.data), self.clock()

        def actuator(self, msg):
            self.obs.actuator, self.obs.actuator_time = msg.data, self.clock()

        def berm_baseline(self,msg):
            self.obs.baseline_ready = bool(msg.data)

        def berm_volume(self,msg):
            self.obs.berm_volume_l, self.obs.berm_time = float(msg.data), self.clock()

        def estop(self, msg):
            self.obs.estop = bool(msg.data)
            self.obs.estop_time = self.clock()

        def start(self, request, response):
            self.obs.now = self.clock()
            self.obs.obstacles = self.lidar_points + self.depth_obstacles
            if self.obstacle_memory:
                self.obs.known_free = self.obstacle_memory.known_free(self.obs.now)
            if not self.armed or self.mission is None:
                response.success, response.message = False, 'disarmed or uncalibrated'
            else:
                out = self.mission.start(self.obs)
                response.success = out.fault == '' and out.phase.value != 'idle'
                response.message = out.fault or out.phase.value
            return response

        def reset(self, request, response):
            if self.mission:
                self.mission.reset()
            response.success, response.message = True, 'manual reset; start still required'
            return response

        def step(self):
            self.obs.now = self.clock()
            if self.obstacle_memory:
                self.lidar_points = self.obstacle_memory.points(self.obs.now)
                self.obs.known_free = self.obstacle_memory.known_free(self.obs.now)
            self.obs.obstacles = self.lidar_points + (self.depth_obstacles if
                self.obs.now - self.obs.depth_time < 0.5 else [])
            out = self.mission.tick(self.obs) if self.armed and self.mission else None
            cmd = Twist()  # zero on every nonmoving state and all faults
            tool = String()
            tool.data = 'stop'
            if out:
                cmd.linear.x, cmd.angular.z = out.linear, out.angular
                tool.data = out.tool
            self.drive.publish(cmd)
            self.tool.publish(tool)
            status = String()
            status.data = ('disarmed' if not self.armed or not self.mission else
                           f'{out.phase.value}:{out.fault}')
            self.state.publish(status)
            if self.obs.now-self.last_visualization >= .5:
                self.publish_visualization()
                self.last_visualization = self.obs.now

        def publish_visualization(self):
            if not self.mission or not self.obstacle_memory:
                return
            cfg = self.mission.cfg
            stamp = self.get_clock().now().to_msg()
            grid = OccupancyGrid()
            grid.header.stamp, grid.header.frame_id = stamp, 'map'
            grid.info.map_load_time = stamp
            grid.info.resolution = cfg.cell
            grid.info.width = math.ceil(cfg.width/cfg.cell)
            grid.info.height = math.ceil(cfg.height/cfg.cell)
            grid.info.origin.orientation.w = 1.0
            free = self.obs.known_free or set()
            hit = set(self.obstacle_memory.cells)
            grid.data = [100 if (i,j) in hit else 0 if (i,j) in free else -1
                         for j in range(grid.info.height)
                         for i in range(grid.info.width)]
            self.grid_pub.publish(grid)
            route = Path()
            route.header.stamp, route.header.frame_id = stamp, 'map'
            for x,y in self.mission.path:
                waypoint = PoseStamped()
                waypoint.header = route.header
                waypoint.pose.position.x, waypoint.pose.position.y = x,y
                waypoint.pose.orientation.w = 1.0
                route.poses.append(waypoint)
            self.path_pub.publish(route)

        def destroy_node(self):
            self.drive.publish(Twist())
            stop = String()
            stop.data = 'stop'
            self.tool.publish(stop)
            super().destroy_node()

    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
