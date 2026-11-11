"""ROS path planner with map-driven replanning around the Stage 1 search."""

from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import math
from threading import Event
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from luna_interfaces.msg import PlannerStatus
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from luna_guidance.grid import GridGeometry, apply_occupancy_patch, occupancy_to_blocked
from luna_guidance.path_check import path_length, project_path, remaining_path
from luna_guidance.replan_policy import should_replan
from luna_guidance.route_search import (
    PlanningMap, SearchSettings, find_route, pose_rejection, remaining_rejection,
)


@dataclass
class SearchJob:
    future: Future
    cancel: Event
    map_version: int
    trigger: str
    position: tuple[float, float]
    requested_at: float


def _timed_search(snapshot, settings, position, yaw, goal, goal_yaw, cancel,
                  allow_escape):
    started = time.perf_counter()
    try:
        route = find_route(
            snapshot, settings, position, yaw, goal, goal_yaw, cancel.is_set,
            allow_escape=allow_escape,
        )
    except ValueError as exc:
        return None, str(exc), time.perf_counter() - started
    return route, '', time.perf_counter() - started


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

        # The supplied YAML selects hybrid. A bare node defaults to circle.
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

        self.rate_hz = self.declare_parameter('rate_hz', 10.0).value
        self.min_replan_interval_s = self.declare_parameter('min_replan_interval_s', 0.5).value
        self.replan_deviation_m = self.declare_parameter('replan_deviation_m', 0.5).value
        self.replan_period_s = self.declare_parameter('replan_period_s', 0.0).value
        self.improvement_ratio = self.declare_parameter('improvement_ratio', 0.9).value
        escape_radius = self.declare_parameter('start_escape_radius_m', 0.3).value
        use_map_updates = self.declare_parameter('use_map_updates', False).value
        for name, value in (
            ('rate_hz', self.rate_hz),
            ('min_replan_interval_s', self.min_replan_interval_s),
            ('replan_deviation_m', self.replan_deviation_m),
            ('replan_period_s', self.replan_period_s),
            ('start_escape_radius_m', escape_radius),
        ):
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative.')
        if self.rate_hz <= 0 or self.replan_deviation_m <= 0:
            raise ValueError('rate_hz and replan_deviation_m must be positive.')
        if not 0 < self.improvement_ratio < 1:
            raise ValueError('improvement_ratio must be between 0 and 1.')
        self.settings = SearchSettings(
            self.footprint_mode, self.robot_radius_m, self.safety_margin_m,
            self.robot_length_m, self.robot_width_m, self.heading_error_deg,
            self.turn_weight, self.max_expansions, self.merge_turnable,
            self.allow_straight_departure, escape_radius,
        )

        self.start_yaw = None
        self.position = None
        self.pose_received_at = None
        self.snapshot = None
        self.map_data = None
        self.base_map = None
        self.map_version = 0
        self.map_error = None
        self.pending_map = None
        self.pending_updates = deque()
        self.pending_goal = None
        self.goal = None
        self.route = None
        self.progress = None
        self.progress_position = None
        self.pending_plan = None
        self.blocked_replan_pending = False
        self.last_attempt = None
        self.input_error = None
        self.job = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='theta_search')

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

        self.create_subscription(
            OccupancyGrid, '/map', self.on_map, self.latched_qos
        )
        if use_map_updates:
            update_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                OccupancyGridUpdate, '/map_updates', self.on_map_update, update_qos
            )
        self.create_subscription(
            Odometry, '/odometry/filtered', self.on_odom, 10
        )
        self.create_subscription(
            PoseStamped, '/goal_pose', self.on_goal, 1
        )

        self.clear_path()
        self.publish_status(PlannerStatus.IDLE, 'Waiting for a goal.')
        self._map_ready = self.create_guard_condition(self._on_map_ready)
        self._search_ready = self.create_guard_condition(self._on_search_ready)
        self.create_timer(1.0 / self.rate_hz, self.on_timer)

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
        self.pending_map = (msg, time.perf_counter())
        self._map_ready.trigger()

    def on_map_update(self, msg):
        # The simulator retains ten patches. Allow that replay to arrive
        # before the latched base map, since the topics are independent.
        if self.snapshot is None and self.pending_map is None:
            if len(self.pending_updates) >= 10:
                self.pending_updates.popleft()
        self.pending_updates.append((msg, time.perf_counter()))
        self._map_ready.trigger()

    def _consume_map(self):
        if self.pending_map is None:
            if self.snapshot is None or not self.pending_updates:
                return None

        full, self.pending_map = self.pending_map, None
        updates = list(self.pending_updates)
        self.pending_updates.clear()
        old = self.snapshot
        geometry = old.geometry if old else None
        data = self.map_data
        base = self.base_map
        received = [t for _, t in updates]

        try:
            if full is not None:
                msg, stamp = full
                if msg.header.frame_id != self.map_frame:
                    raise ValueError(f'Map frame must be {self.map_frame!r}.')
                origin = msg.info.origin
                geometry = GridGeometry(
                    msg.info.width, msg.info.height, msg.info.resolution,
                    origin.position.x, origin.position.y,
                    self.planar_yaw(origin.orientation),
                )
                incoming = (geometry, tuple(msg.data))
                if incoming != base or old is None:
                    base = incoming
                    data = incoming[1]
                    if old is not None:
                        updates = [(m, t) for m, t in updates if t >= stamp]
                    received = [stamp, *(t for _, t in updates)]

            for msg, _ in updates:
                if msg.header.frame_id != self.map_frame:
                    raise ValueError(f'Map update frame must be {self.map_frame!r}.')
                data = apply_occupancy_patch(
                    data, geometry, msg.x, msg.y, msg.width, msg.height, msg.data,
                )
            if old is not None and geometry == old.geometry and data == self.map_data:
                self.base_map = base
                return None

            blocked = occupancy_to_blocked(
                data, geometry, self.occupied_threshold, self.unknown_is_blocked,
            )
            self.map_data, self.base_map = data, base
            self.map_error = None
            if old is not None and geometry == old.geometry and blocked == old.raw_blocked:
                return None

            checker = self.settings.checker(blocked, geometry)
            self.snapshot = PlanningMap(geometry, blocked, checker)
        except ValueError as exc:
            self.snapshot = self.map_data = self.base_map = None
            self.map_error = f'Map rejected: {exc}'
            self.get_logger().error(self.map_error)

        self.map_version += 1
        geometry_changed = old is not None and (
            self.snapshot is None or old.geometry != self.snapshot.geometry
        )
        if geometry_changed or self.snapshot is None:
            self._cancel_search()
        return geometry_changed, min(received, default=time.perf_counter())

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

    def on_goal(self, msg):
        self.pending_goal = (msg, time.perf_counter())

    def _consume_goal(self):
        if self.pending_goal is None:
            return
        (msg, received), self.pending_goal = self.pending_goal, None
        self._cancel_search()
        self._hold_route(force=True)
        self.goal = self.pending_plan = self.input_error = None
        self.blocked_replan_pending = False
        try:
            if msg.header.frame_id != self.map_frame:
                raise ValueError(f'Goal frame must be {self.map_frame!r}.')
            goal = (msg.pose.position.x, msg.pose.position.y)
            if not all(math.isfinite(value) for value in goal):
                raise ValueError('Goal position must be finite.')
            yaw = self.planar_yaw(msg.pose.orientation)
        except ValueError as exc:
            self.publish_status(PlannerStatus.FAILED, str(exc))
            return
        self.goal = goal, yaw
        self._request_plan('new goal', received)

    def _cancel_search(self):
        if self.job is not None:
            self.job.cancel.set()

    def _hold_route(self, *, force=False):
        if self.route is not None or force:
            self.clear_path()
        self.route = self.progress = self.progress_position = None

    def _request_plan(self, reason, received=None):
        if self.pending_plan is None or reason == 'new goal':
            self.pending_plan = (reason, time.perf_counter() if received is None else received)

    def _hold_for_replan(self, reason, received=None):
        if reason == 'path blocked' and self.route is not None:
            self.blocked_replan_pending = True
        self._cancel_search()
        self._hold_route()
        self._request_plan(reason, received)
        self.publish_status(PlannerStatus.PLANNING, f'replan: {reason}; holding.')

    def _pose_error(self, now):
        if self.position is None or self.pose_received_at is None:
            return 'No valid odometry received.'
        age = (now - self.pose_received_at).nanoseconds * 1e-9
        if age < 0 or age > self.odom_timeout_s:
            return 'Odometry is stale or the clock moved backward.'
        return None

    def _route_rejection(self, route, projection, start_yaw=None):
        checker = self.snapshot.checker
        if route.escape and projection.segment == 0 and projection.fraction < 1.0:
            checker = self.settings.checker(
                self.snapshot.raw_blocked, self.snapshot.geometry, escape=True,
            )
        reason = pose_rejection(checker, self.position, self.start_yaw)
        if reason is not None:
            return reason
        return remaining_rejection(
            self.snapshot, self.settings, route, projection, self.goal[1], start_yaw,
        )

    def on_timer(self):
        self._process_planning(schedule_search=True)

    def _on_map_ready(self):
        if self.pending_map is not None or self.pending_updates:
            self._process_planning(schedule_search=True)

    def _process_planning(self, *, schedule_search):
        self._consume_goal()
        changed = self._consume_map()
        now = self.get_clock().now()
        if self.goal is None:
            if changed is not None and self.map_error is not None:
                self.publish_status(PlannerStatus.FAILED, self.map_error)
            self._complete_search()
            return

        if changed is not None:
            geometry_changed, received = changed
            if self.route is not None and geometry_changed:
                self._hold_for_replan('map geometry changed', received)
            elif self.route is None and (self.job is None or self.job.cancel.is_set()):
                trigger = 'map changed'
                if self.job is not None and self.job.trigger not in ('new goal', 'periodic'):
                    trigger = self.job.trigger
                    received = min(received, self.job.requested_at)
                self._request_plan(trigger, received)

        error = (
            (self.map_error or 'No valid map received.')
            if self.snapshot is None else self._pose_error(now)
        )
        if error is not None:
            self._cancel_search()
            self._hold_route()
            self._request_plan('inputs restored')
            if self.input_error != error:
                self.publish_status(PlannerStatus.FAILED, error)
                self.input_error = error
            self._complete_search()
            return
        self.input_error = None

        if self.route is not None:
            projection = self.progress
            if self.position != self.progress_position:
                projection = project_path(
                    self.route.points, self.position, self.progress,
                    max_advance_m=self.replan_deviation_m,
                )
            if changed is not None and self._route_rejection(self.route, projection) is not None:
                self._hold_for_replan('path blocked', changed[1])
            elif projection.deviation > self.replan_deviation_m:
                self._hold_for_replan('path deviation')
            else:
                self.progress = projection
                self.progress_position = self.position

        self._complete_search()
        if self.job is not None or not schedule_search:
            return
        now_s = now.nanoseconds * 1e-9
        pending_reason = self.pending_plan[0] if self.pending_plan else None
        trigger = should_replan(
            now_s, self.last_attempt, pending_reason, self.min_replan_interval_s,
            have_path=self.route is not None, period=self.replan_period_s,
            urgent=self.blocked_replan_pending,
        )
        if trigger is not None:
            self._start_search(trigger, now_s)

    def _start_search(self, trigger, now_s):
        requested_at = self.pending_plan[1] if self.pending_plan else time.perf_counter()
        self.pending_plan = None
        self.blocked_replan_pending = False
        self.last_attempt = now_s
        label = 'Processing new goal.' if trigger == 'new goal' else f'replan: {trigger}'
        self.publish_status(PlannerStatus.PLANNING, label)
        cancel = Event()
        future = self.worker.submit(
            _timed_search, self.snapshot, self.settings, self.position,
            self.start_yaw, *self.goal, cancel, trigger != 'new goal',
        )
        self.job = SearchJob(
            future, cancel, self.map_version, trigger, self.position, requested_at,
        )
        future.add_done_callback(self._notify_search_ready)

    def _notify_search_ready(self, _future):
        # The worker only wakes the executor. ROS state and publishers stay
        # in the node's callback group.
        if self.context.ok():
            try:
                self._search_ready.trigger()
            except RuntimeError as exc:
                if self.context.ok():
                    self.get_logger().warning(f'Could not wake the search callback: {exc}')

    def _on_search_ready(self):
        if self.job is not None and self.job.future.done():
            # Consume pending map and goal messages before accepting a result.
            # Map and timer callbacks decide when to start searches.
            self._process_planning(schedule_search=False)

    def _complete_search(self):
        job = self.job
        if job is None or not job.future.done():
            return
        self.job = None
        if job.cancel.is_set() or self.goal is None:
            return

        map_changed = job.map_version != self.map_version
        retry_reason = job.trigger
        if job.trigger == 'new goal':
            retry_reason = 'map changed' if map_changed else 'start changed'
        label = '' if job.trigger == 'new goal' else f'replan: {job.trigger}; '
        try:
            route, error, elapsed = job.future.result()
        except Exception as exc:
            route, error, elapsed = None, f'Search failed: {exc}', 0.0
        if route is None:
            if map_changed and job.trigger != 'periodic':
                self._request_plan(retry_reason, job.requested_at)
                error = 'Map changed during search: ' + error
            self._report_search_failure(label + error, elapsed)
            return

        projection = project_path(route.points, route.points[0])
        if math.dist(job.position, self.position) > self.snapshot.geometry.resolution:
            rejection = 'Start pose changed during search.'
        else:
            rejection = self._route_rejection(route, projection, self.start_yaw)
            if rejection is not None and map_changed:
                rejection = 'Map changed during search: ' + rejection
        if rejection is not None:
            if job.trigger != 'periodic':
                self._request_plan(retry_reason, job.requested_at)
            self._report_search_failure(label + rejection, elapsed)
            return

        if job.trigger == 'periodic' and self.route is not None:
            remaining = path_length(remaining_path(self.route.points, self.progress))
            if path_length(route.points) >= self.improvement_ratio * remaining:
                self.publish_status(
                    PlannerStatus.SUCCEEDED, label + 'retained current path.',
                    path_length_m=remaining, planning_time_s=elapsed,
                )
                return

        path, length = self.make_path(
            route.points, self.goal[1], self.snapshot.geometry.resolution,
        )
        self.route, self.progress = route, projection
        self.progress_position = job.position
        self.path_pub.publish(path)
        self.publish_status(
            PlannerStatus.SUCCEEDED, label + 'Path found.',
            path_length_m=length, planning_time_s=elapsed,
        )
        response = time.perf_counter() - job.requested_at
        self.get_logger().info(
            f'Path published: {length:.2f} m, {len(path.poses)} poses, '
            f'search {elapsed:.3f} s, {job.trigger} response {response:.3f} s'
        )

    def _report_search_failure(self, reason, elapsed):
        if self.route is not None:
            length = path_length(remaining_path(self.route.points, self.progress))
            self.publish_status(
                PlannerStatus.SUCCEEDED, reason + ' Retained current path.',
                path_length_m=length, planning_time_s=elapsed,
            )
        else:
            self.publish_status(PlannerStatus.FAILED, reason, planning_time_s=elapsed)
            self.get_logger().warning(f'Planning failed: {reason}')

    def destroy_node(self):
        self._cancel_search()
        self.worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()

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
