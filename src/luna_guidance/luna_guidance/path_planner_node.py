"""ROS path planner with circular or heading-aware hybrid clearance."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from luna_interfaces.msg import PlannerStatus
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from luna_guidance.clearance import CircularClearance
from luna_guidance.grid import GridGeometry, occupancy_to_blocked
from luna_guidance.heading_clearance import CircleFirstHeadingClearance
from luna_guidance.heading_theta_star import heading_theta_star
from luna_guidance.theta_star import simplify_path, theta_star


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.map_frame = self.declare_parameter(
            'map_frame', 'map'
        ).value
        self.robot_radius_m = self.declare_parameter(
            'robot_radius_m', 0.715
        ).value
        self.safety_margin_m = self.declare_parameter(
            'safety_margin_m', 0.1
        ).value
        self.occupied_threshold = self.declare_parameter(
            'occupied_threshold', 50
        ).value
        self.unknown_is_blocked = self.declare_parameter(
            'unknown_is_blocked', True
        ).value
        self.odom_timeout_s = self.declare_parameter(
            'odom_timeout_s', 0.5
        ).value

        for name, value in (
            ('robot_radius_m', self.robot_radius_m),
            ('safety_margin_m', self.safety_margin_m),
        ):
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative.')

        if not math.isfinite(self.odom_timeout_s) or self.odom_timeout_s <= 0:
            raise ValueError('odom_timeout_s must be finite and positive.')
        if not 1 <= self.occupied_threshold <= 100:
            raise ValueError('occupied_threshold must be between 1 and 100.')
        if not self.map_frame:
            raise ValueError('map_frame must not be empty.')

        # The supplied YAML selects hybrid mode; standalone defaults to circle.
        self.footprint_mode = self.declare_parameter(
            'footprint_mode', 'circle'
        ).value
        self.robot_length_m = self.declare_parameter(
            'robot_length_m', 1.2319
        ).value
        self.robot_width_m = self.declare_parameter(
            'robot_width_m', 0.7239
        ).value
        self.heading_error_deg = self.declare_parameter(
            'heading_error_deg', 5.0
        ).value
        self.turn_weight = self.declare_parameter(
            'turn_weight', 0.0
        ).value
        self.max_expansions = self.declare_parameter(
            'max_expansions', 20000
        ).value
        self.merge_turnable = self.declare_parameter(
            'merge_turnable', True
        ).value
        self.allow_straight_departure = self.declare_parameter(
            'allow_straight_departure', False
        ).value

        if self.footprint_mode not in ('circle', 'hybrid'):
            raise ValueError('footprint_mode must be circle or hybrid.')

        for name, value in (
            ('robot_length_m', self.robot_length_m),
            ('robot_width_m', self.robot_width_m),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive.')

        if (
            not math.isfinite(self.heading_error_deg)
            or not 0 <= self.heading_error_deg <= 180
        ):
            raise ValueError('heading_error_deg must be between 0 and 180.')
        if not math.isfinite(self.turn_weight) or self.turn_weight < 0:
            raise ValueError('turn_weight must be finite and nonnegative.')
        if type(self.max_expansions) is not int or self.max_expansions <= 0:
            raise ValueError('max_expansions must be a positive integer.')
        if type(self.merge_turnable) is not bool:
            raise ValueError('merge_turnable must be Boolean.')
        if type(self.allow_straight_departure) is not bool:
            raise ValueError('allow_straight_departure must be Boolean.')
        if self.merge_turnable and self.turn_weight != 0:
            raise ValueError('Merging requires zero turn_weight.')
        if self.footprint_mode == 'hybrid':
            corner_radius = math.hypot(
                self.robot_length_m / 2, self.robot_width_m / 2
            )
            if self.robot_radius_m < corner_radius:
                raise ValueError('robot_radius_m must enclose the footprint.')

        self.start_yaw = None

        self.geometry = None
        self.clearance_checker = None
        self.blocked = None
        self.position = None
        self.pose_received_at = None

        self.latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.path_pub = self.create_publisher(
            Path, '/guidance/path', self.latched_qos
        )
        self.status_pub = self.create_publisher(
            PlannerStatus, '/planning/status', self.latched_qos
        )

        self.last_map = None

        self.create_subscription(
            OccupancyGrid, '/map', self.on_map, self.latched_qos
        )
        self.create_subscription(
            Odometry, '/odometry/filtered', self.on_odom, 10
        )
        self.create_subscription(
            PoseStamped, '/goal_pose', self.on_goal, 1
        )

        self.clear_path()
        self.publish_status(PlannerStatus.IDLE, 'Waiting for a goal.')

    def publish_status(self, state, message, path_length_m=0.0,
                       planning_time_s=0.0):
        status = PlannerStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.map_frame
        status.state = state
        status.message = message
        status.path_length_m = float(path_length_m)
        status.planning_time_s = float(planning_time_s)
        self.status_pub.publish(status)

    def clear_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        self.path_pub.publish(path)

    @staticmethod
    def planar_yaw(q):
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Quaternion contains nonfinite values.')

        norm = math.hypot(*values)
        if norm < 1e-6:
            raise ValueError('Quaternion has zero length.')

        x, y, z, w = (value / norm for value in values)
        if abs(x) > 1e-6 or abs(y) > 1e-6:
            raise ValueError('Expected a planar orientation with yaw only.')

        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def on_map(self, msg):
        # Ignore repeated publications of identical map contents.
        previous = self.last_map
        if previous is not None:
            if (
                msg.header.frame_id == previous.header.frame_id
                and msg.info == previous.info
                and msg.data == previous.data
            ):
                return

        self.clear_path()
        self.geometry = None
        self.clearance_checker = None
        self.blocked = None
        self.last_map = None

        try:
            if msg.header.frame_id != self.map_frame:
                raise ValueError(
                    f'Map frame must be {self.map_frame!r}.'
                )

            origin = msg.info.origin
            yaw = self.planar_yaw(origin.orientation)
            geometry = GridGeometry(
                width=msg.info.width,
                height=msg.info.height,
                resolution=msg.info.resolution,
                origin_x=origin.position.x,
                origin_y=origin.position.y,
                origin_yaw=yaw,
            )

            blocked = occupancy_to_blocked(
                msg.data,
                geometry,
                occupied_threshold=self.occupied_threshold,
                unknown_is_blocked=self.unknown_is_blocked,
            )
            if self.footprint_mode == 'hybrid':
                checker = CircleFirstHeadingClearance(
                    blocked,
                    geometry,
                    length_m=self.robot_length_m,
                    width_m=self.robot_width_m,
                    margin_m=self.safety_margin_m,
                    heading_error_deg=self.heading_error_deg,
                    turn_radius_m=self.robot_radius_m,
                )
                # Hybrid feasibility depends on heading, so do not
                # pre-reject cells using a circle-inflated grid.
                center_blocked = blocked
            else:
                checker = CircularClearance(
                    blocked,
                    geometry,
                    self.robot_radius_m,
                    self.safety_margin_m,
                )
                center_blocked = checker.center_blocked_grid()
        except ValueError as exc:
            reason = f'Map rejected: {exc}'
            self.publish_status(PlannerStatus.FAILED, reason)
            self.get_logger().error(reason)
            return

        self.geometry = geometry
        self.blocked = center_blocked
        self.clearance_checker = checker
        self.last_map = msg
        self.publish_status(
            PlannerStatus.IDLE, 'Map loaded; send a new goal.'
        )
        self.get_logger().info(
            f'Map loaded: {geometry.width} x {geometry.height}, '
            f'{geometry.resolution:.3f} m/cell'
        )

    def on_odom(self, msg):
        position = msg.pose.pose.position
        try:
            if msg.header.frame_id != self.map_frame:
                raise ValueError('Wrong odometry frame.')
            if not all(math.isfinite(v) for v in (position.x, position.y)):
                raise ValueError('Nonfinite odometry position.')

            yaw = None
            if self.footprint_mode == 'hybrid':
                yaw = self.planar_yaw(msg.pose.pose.orientation)
        except ValueError:
            self.position = None
            self.start_yaw = None
            self.pose_received_at = None
            return

        self.position = (position.x, position.y)
        self.start_yaw = yaw
        self.pose_received_at = self.get_clock().now()

    def plan_hybrid(self, goal, goal_yaw):
        """
        Plan and recheck straight segments and stationary turns.

        The search leaves goal heading unconstrained. A requested final
        rotation is checked afterward; rejection does not prove that no
        alternative arrival heading could work.
        """
        if self.start_yaw is None:
            raise ValueError('No valid starting orientation received.')

        checker = self.clearance_checker
        result = heading_theta_star(
            checker,
            self.position,
            goal,
            self.start_yaw,
            turn_weight=self.turn_weight,
            max_expansions=self.max_expansions,
            merge_turnable=self.merge_turnable,
            allow_straight_departure=self.allow_straight_departure,
        )

        if result['status'] != 'PATH_FOUND':
            raise ValueError(f"Hybrid search: {result['status']}")

        points = result['path']
        if (
            not points
            or tuple(points[0]) != tuple(self.position)
            or tuple(points[-1]) != tuple(goal)
        ):
            raise ValueError('Hybrid path has incorrect endpoints.')

        def heading_change(a, b):
            return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

        yaw = self.start_yaw
        if not checker.pose_is_free(points[0], yaw):
            raise ValueError('Final recheck rejected the starting footprint.')

        for a, b in zip(points, points[1:]):
            if math.dist(a, b) <= 1e-12:
                raise ValueError('Hybrid path contains a zero-length segment.')

            departure = math.atan2(b[1] - a[1], b[0] - a[0])
            if (
                heading_change(yaw, departure) > 1e-9
                and not checker.turn_is_free(a)
            ):
                raise ValueError('Final recheck rejected a stationary turn.')
            if not checker.straight_is_free(a, b):
                raise ValueError('Final recheck rejected a straight segment.')
            yaw = departure

        if (
            heading_change(yaw, goal_yaw) > 1e-9
            and not checker.turn_is_free(goal)
        ):
            raise ValueError(
                'GOAL_TURN_REJECTED: requested final rotation lacks clearance; '
                'alternative arrival headings were not searched.'
            )

        return points

    def plan_circle(self, goal):
        """Plan and recheck circular clearance, including exact endpoints."""
        geometry = self.geometry
        checker = self.clearance_checker
        start_world = self.position

        for name, position in (
            ('Start', start_world),
            ('Goal', goal),
        ):
            reason = checker.point_reason(position)
            if reason is not None:
                raise ValueError(f'{name}: {reason}')

        # Prefer the direct route when the full segment is clear.
        if checker.segment_is_free(start_world, goal):
            points = [start_world, goal]
        else:
            start_cell = geometry.world_to_cell(*start_world)
            goal_cell = geometry.world_to_cell(*goal)

            for name, cell, position in (
                ('Start', start_cell, start_world),
                ('Goal', goal_cell, goal),
            ):
                if not geometry.contains(cell):
                    raise ValueError(f'{name} lies outside the map.')

                x, y = cell
                if self.blocked[y][x]:
                    raise ValueError(
                        f'{name} position is valid, but its grid-cell '
                        'center lacks clearance. Grid attachment failed.'
                    )

                center = geometry.cell_to_world(cell)
                if not checker.segment_is_free(position, center):
                    raise ValueError(
                        f'{name} cannot connect safely to its grid-cell '
                        'center. Grid attachment failed.'
                    )

            cells = theta_star(
                self.blocked,
                start_cell,
                goal_cell,
                segment_is_free=checker.cell_segment_is_free,
            )
            if not cells:
                raise ValueError(
                    'Search found no route on the circular-clearance '
                    'grid.'
                )

            points = [start_world]
            points.extend(
                geometry.cell_to_world(cell)
                for cell in simplify_path(cells)
            )
            points.append(goal)

        # Validate the final geometry, including exact endpoints,
        # before sampling it into a ROS Path.
        for a, b in zip(points, points[1:]):
            if not checker.segment_is_free(a, b):
                raise ValueError(
                    'Final path segment failed circular clearance.'
                )


        return points

    def on_goal(self, msg):
        started = time.perf_counter()
        self.clear_path()
        self.publish_status(PlannerStatus.PLANNING, 'Processing new goal.')

        try:
            if msg.header.frame_id != self.map_frame:
                raise ValueError(
                    f'Goal frame must be {self.map_frame!r}.'
                )
            if (
                self.geometry is None
                or self.blocked is None
                or self.clearance_checker is None
            ):
                raise ValueError('No valid map received.')
            if self.position is None or self.pose_received_at is None:
                raise ValueError('No valid odometry received.')

            age = (
                self.get_clock().now() - self.pose_received_at
            ).nanoseconds * 1e-9
            if age < 0 or age > self.odom_timeout_s:
                raise ValueError('Odometry is stale or the clock moved backward.')

            goal = (msg.pose.position.x, msg.pose.position.y)
            if not all(math.isfinite(value) for value in goal):
                raise ValueError('Goal position must be finite.')
            goal_yaw = self.planar_yaw(msg.pose.orientation)

            geometry = self.geometry

            if self.footprint_mode == 'hybrid':
                points = self.plan_hybrid(goal, goal_yaw)
            else:
                points = self.plan_circle(goal)

            path, length = self.make_path(
                points, goal_yaw, geometry.resolution
            )
        except ValueError as exc:
            elapsed = time.perf_counter() - started
            self.publish_status(
                PlannerStatus.FAILED,
                str(exc),
                planning_time_s=elapsed,
            )
            self.get_logger().warning(f'Planning failed: {exc}')
            return

        elapsed = time.perf_counter() - started
        self.path_pub.publish(path)
        self.publish_status(
            PlannerStatus.SUCCEEDED,
            'Path found.',
            path_length_m=length,
            planning_time_s=elapsed,
        )
        self.get_logger().info(
            f'Path published: {length:.2f} m, '
            f'{len(path.poses)} poses, {elapsed:.3f} s'
        )

    def make_path(self, points, goal_yaw, spacing):
        """Sample each segment without changing its geometry."""
        sampled = [points[0]]
        length = 0.0

        for a, b in zip(points, points[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            segment_length = math.hypot(dx, dy)
            if segment_length == 0:
                continue

            length += segment_length
            steps = max(1, math.ceil(segment_length / spacing))
            for step in range(1, steps + 1):
                if step == steps:
                    sampled.append(b)
                else:
                    fraction = step / steps
                    sampled.append((
                        a[0] + fraction * dx,
                        a[1] + fraction * dy,
                    ))

        # The controller reads goal orientation only with at least two poses.
        if len(sampled) == 1:
            sampled.append(sampled[0])

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame

        for index, (x, y) in enumerate(sampled):
            yaw = goal_yaw
            if index < len(sampled) - 1:
                next_x, next_y = sampled[index + 1]
                if (next_x, next_y) != (x, y):
                    yaw = math.atan2(next_y - y, next_x - x)

            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)

        return path, length


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
