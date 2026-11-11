"""Direct callback tests with mocked publishers."""

from contextlib import contextmanager
from copy import deepcopy
import math
from threading import Event
import unittest
from unittest.mock import Mock, patch

import rclpy
from geometry_msgs.msg import PoseStamped
from luna_interfaces.msg import PlannerStatus
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.time import Time

from luna_guidance.path_planner_node import PathPlannerNode
from luna_guidance.route_search import find_route


class TestPathPlannerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[
            '--ros-args',
            '-r', '/guidance/path:=/planner_test/path',
            '-r', '/planning/status:=/planner_test/status',
        ])

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.node = PathPlannerNode()
        self.addCleanup(self.node.destroy_node)

        # Capture callback outputs without sending them to a controller.
        self.node.path_pub = Mock()
        self.node.status_pub = Mock()
        self.clock_ns = 0
        clock = Mock()
        clock.now.side_effect = lambda: Time(nanoseconds=self.clock_ns)
        self.node.get_clock = Mock(return_value=clock)

        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.width = 60
        self.map.info.height = 60
        self.map.info.resolution = 0.1
        self.map.info.origin.orientation.w = 1.0
        self.map.data = [0] * 3600
        self.node.on_map(self.map)
        self.node.on_timer()

        self.odom = Odometry()
        self.odom.header.frame_id = 'map'
        self.odom.pose.pose.position.x = 1.05
        self.odom.pose.pose.position.y = 1.05
        self.odom.pose.pose.orientation.w = 1.0

        self.goal = PoseStamped()
        self.goal.header.frame_id = 'map'
        self.goal.pose.position.x = 4.05
        self.goal.pose.position.y = 4.05
        self.goal.pose.orientation.z = math.sin(math.pi / 4)
        self.goal.pose.orientation.w = math.cos(math.pi / 4)

    def plan(self):
        self.node.on_odom(self.odom)
        self.node.on_goal(self.goal)
        self.node.on_timer()
        return self.finish_plan()

    def finish_plan(self):
        if self.node.job is not None:
            self.node.job.future.result(timeout=5)
            self.node.on_timer()
        path = self.node.path_pub.publish.call_args.args[0]
        status = self.node.status_pub.publish.call_args.args[0]
        return path, status

    def set_pose_time(self, seconds, x=None, y=None):
        self.clock_ns = round(seconds * 1e9)
        if x is not None:
            self.odom.pose.pose.position.x = x
        if y is not None:
            self.odom.pose.pose.position.y = y
        self.node.on_odom(self.odom)

    def plan_horizontal(self):
        self.odom.pose.pose.position.y = 2.05
        self.goal.pose.position.x = 4.95
        self.goal.pose.position.y = 2.05
        return self.plan()

    def map_with_obstacle(self, x, y):
        grid = deepcopy(self.map)
        grid.data[y * 60 + x] = 100
        return grid

    @contextmanager
    def pause_search(self):
        """Deliver updates while the worker still owns its original snapshot."""
        started, release = Event(), Event()

        def slow_search(snapshot, settings, position, yaw, goal, goal_yaw, cancel,
                        *, allow_escape):
            started.set()
            if not release.wait(5):
                raise RuntimeError('Test did not release search.')
            return find_route(snapshot, settings, position, yaw, goal, goal_yaw,
                              lambda: False, allow_escape=allow_escape)

        with patch('luna_guidance.path_planner_node.find_route', side_effect=slow_search):
            try:
                yield started, release
            finally:
                release.set()

    def test_successful_path(self):
        path, status = self.plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertEqual(path.header.frame_id, 'map')
        self.assertGreater(len(path.poses), 2)

        first = path.poses[0].pose.position
        last = path.poses[-1].pose.position
        self.assertAlmostEqual(first.x, 1.05)
        self.assertAlmostEqual(first.y, 1.05)
        self.assertAlmostEqual(last.x, 4.05)
        self.assertAlmostEqual(last.y, 4.05)

        measured_length = 0.0
        for a, b in zip(path.poses, path.poses[1:]):
            distance = math.hypot(
                b.pose.position.x - a.pose.position.x,
                b.pose.position.y - a.pose.position.y,
            )
            self.assertLessEqual(distance, 0.100001)
            measured_length += distance

        self.assertAlmostEqual(
            status.path_length_m, measured_length, places=5
        )
        self.assertGreaterEqual(status.planning_time_s, 0.0)

        orientation = path.poses[-1].pose.orientation
        self.assertAlmostEqual(orientation.z, math.sin(math.pi / 4))
        self.assertAlmostEqual(orientation.w, math.cos(math.pi / 4))

    def test_invalid_goal_clears_previous_path(self):
        _, status = self.plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)

        self.goal.pose.position.x = -1.0
        path, status = self.plan()
        self.assertEqual(status.state, PlannerStatus.FAILED)
        self.assertIn('outside', status.message)
        self.assertEqual(path.poses, [])

    def test_wrong_goal_frame(self):
        self.goal.header.frame_id = 'odom'
        path, status = self.plan()
        self.assertEqual(status.state, PlannerStatus.FAILED)
        self.assertIn('frame', status.message.lower())
        self.assertEqual(path.poses, [])

    def test_same_position_goal(self):
        self.goal.pose.position.x = 1.05
        self.goal.pose.position.y = 1.05
        path, status = self.plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertAlmostEqual(status.path_length_m, 0.0)
        self.assertEqual(len(path.poses), 2)
        for pose in path.poses:
            self.assertAlmostEqual(pose.pose.position.x, 1.05)
            self.assertAlmostEqual(pose.pose.position.y, 1.05)
        self.assertAlmostEqual(
            path.poses[-1].pose.orientation.z, math.sin(math.pi / 4)
        )

    def load_coarse_map(self, with_obstacle=False):
        """Load coarse geometry to expose endpoint attachment cases."""
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = 10
        grid.info.height = 10
        grid.info.resolution = 1.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 100
        if with_obstacle:
            grid.data[4 * 10 + 4] = 100
        self.node.on_map(grid)
        self.node.on_timer()

    def test_exact_start_rejected_even_when_cell_center_is_free(self):
        # Copy rather than mutate the map retained by the node.
        grid = deepcopy(self.map)
        grid.data[30 * 60 + 30] = 100
        self.node.on_map(grid)
        self.node.on_timer()

        # Obstacle begins at x=3.0 m. Exact start gap is about 0.81 m,
        # below 0.815 m, while its cell-center gap is about 0.85 m.
        self.odom.pose.pose.position.x = 2.19
        self.odom.pose.pose.position.y = 3.05
        snapshot = self.node.snapshot
        cell = snapshot.geometry.world_to_cell(2.19, 3.05)
        self.assertIsNone(snapshot.checker.point_reason(snapshot.geometry.cell_to_world(cell)))

        path, status = self.plan()
        self.assertEqual(status.state, PlannerStatus.FAILED)
        self.assertIn('Start:', status.message)
        self.assertIn('obstacle clearance', status.message)
        self.assertEqual(path.poses, [])

    def test_direct_path_does_not_require_free_cell_centers(self):
        self.load_coarse_map()
        self.odom.pose.pose.position.x = 0.9
        self.odom.pose.pose.position.y = 2.0
        self.goal.pose.position.x = 0.9
        self.goal.pose.position.y = 7.0

        # Exact x=0.9 is safe, although this coarse cell's x=0.5
        # center is too close to the map boundary.
        checker = self.node.snapshot.checker
        self.assertIsNotNone(checker.point_reason((0.5, 2.5)))

        path, status = self.plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertAlmostEqual(status.path_length_m, 5.0)
        self.assertAlmostEqual(path.poses[0].pose.position.y, 2.0)
        self.assertAlmostEqual(path.poses[-1].pose.position.y, 7.0)
        for pose in path.poses:
            self.assertAlmostEqual(pose.pose.position.x, 0.9)

    def test_grid_attachment_failure_has_specific_reason(self):
        self.load_coarse_map(with_obstacle=True)
        self.odom.pose.pose.position.x = 0.9
        self.odom.pose.pose.position.y = 4.5
        self.goal.pose.position.x = 8.0
        self.goal.pose.position.y = 4.5

        # Both exact endpoints are valid, but the direct segment crosses
        # an obstacle and the start's own cell center is unavailable.
        checker = self.node.snapshot.checker
        self.assertIsNone(checker.point_reason((0.9, 4.5)))
        self.assertIsNone(checker.point_reason((8.0, 4.5)))
        self.assertFalse(
            checker.segment_is_free((0.9, 4.5), (8.0, 4.5))
        )

        path, status = self.plan()
        self.assertEqual(status.state, PlannerStatus.FAILED)
        self.assertIn('Start position is valid', status.message)
        self.assertIn('Grid attachment failed', status.message)
        self.assertEqual(path.poses, [])

    def test_mock_patch_starts_blocked_replan_from_latest_odometry_immediately(self):
        _, original = self.plan_horizontal()
        self.assertEqual(original.state, PlannerStatus.SUCCEEDED)
        self.set_pose_time(0.1, x=1.25)
        update = OccupancyGridUpdate()
        update.header.frame_id = 'map'
        update.x, update.y = 30, 20
        update.width = update.height = 1
        update.data = [100]
        with patch.object(self.node._map_ready, 'trigger') as wake:
            self.node.on_map_update(update)
            wake.assert_called_once_with()
        self.assertGreater(len(self.node.path_pub.publish.call_args.args[0].poses), 2)
        self.set_pose_time(0.1, x=1.35)
        self.node._on_map_ready()

        self.assertIsNotNone(self.node.job)
        self.assertAlmostEqual(self.node.last_attempt, 0.1)
        self.assertEqual(self.node.path_pub.publish.call_args.args[0].poses, [])
        self.assertIn('path blocked', self.node.status_pub.publish.call_args.args[0].message)

        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertIn('path blocked', status.message)
        self.assertAlmostEqual(path.poses[0].pose.position.x, 1.35)
        self.assertAlmostEqual(path.poses[-1].pose.position.x, 4.95)
        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        checker = self.node.snapshot.checker
        self.assertTrue(all(checker.segment_is_free(a, b) for a, b in zip(points, points[1:])))

    def test_full_map_starts_each_newly_blocked_route_during_the_interval(self):
        path, _ = self.plan_horizontal()
        grid = deepcopy(self.map)
        for seconds in (0.1, 0.2):
            middle = path.poses[len(path.poses) // 2].pose.position
            x, y = self.node.snapshot.geometry.world_to_cell(middle.x, middle.y)
            grid.data[y * grid.info.width + x] = 100
            self.set_pose_time(seconds)
            self.node.on_map(grid)
            self.node._on_map_ready()

            self.assertEqual(self.node.path_pub.publish.call_args.args[0].poses, [])
            self.assertIsNotNone(self.node.job)
            self.assertEqual(self.node.job.trigger, 'path blocked')
            self.assertAlmostEqual(self.node.last_attempt, seconds)
            self.assertFalse(self.node.blocked_replan_pending)
            path, status = self.finish_plan()
            self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
            self.assertAlmostEqual(path.poses[-1].pose.position.x, 4.95)
            points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
            checker = self.node.snapshot.checker
            self.assertTrue(all(checker.segment_is_free(a, b)
                                for a, b in zip(points, points[1:])))

    def test_unrelated_map_updates_keep_a_valid_inflight_replan(self):
        self.plan_horizontal()
        with self.pause_search() as (started, release):
            self.set_pose_time(0.1)
            grid = self.map_with_obstacle(30, 20)
            self.node.on_map(grid)
            self.node._on_map_ready()
            self.assertTrue(started.wait(3))
            job = self.node.job

            for i in range(3):
                self.set_pose_time(0.2 + i * 0.05)
                updated = deepcopy(grid)
                updated.data[54 * 60 + 54 + i] = 100
                self.node.on_map(updated)
                self.node._on_map_ready()
                self.assertIs(self.node.job, job)
                self.assertFalse(job.cancel.is_set())
                self.assertEqual(self.node.path_pub.publish.call_args.args[0].poses, [])

            release.set()
            path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertIn('path blocked', status.message)
        self.assertAlmostEqual(path.poses[-1].pose.position.x, 4.95)
        self.assertIsNone(self.node.pending_plan)
        self.assertEqual(self.node.last_attempt, 0.1)

    def test_rejected_blocked_search_retry_respects_the_interval(self):
        self.plan_horizontal()
        with self.pause_search() as (started, release):
            self.set_pose_time(0.1)
            grid = self.map_with_obstacle(30, 20)
            self.node.on_map(grid)
            self.node._on_map_ready()
            self.assertTrue(started.wait(3))
            self.assertFalse(self.node.blocked_replan_pending)

            self.set_pose_time(0.2)
            blocked_goal = deepcopy(grid)
            blocked_goal.data[20 * 60 + 49] = 100
            self.node.on_map(blocked_goal)
            self.node._on_map_ready()
            self.assertFalse(self.node.job.cancel.is_set())
            release.set()
            _, status = self.finish_plan()
            self.assertEqual(status.state, PlannerStatus.FAILED)
            self.assertIn('Map changed during search', status.message)
            self.assertIsNone(self.node.job)
            self.assertEqual(self.node.pending_plan[0], 'path blocked')
            self.assertEqual(self.node.path_pub.publish.call_args.args[0].poses, [])

        self.set_pose_time(0.599)
        self.node.on_map(grid)
        self.node.on_timer()
        self.assertIsNone(self.node.job)
        self.set_pose_time(0.6)
        self.node.on_timer()
        self.assertIsNotNone(self.node.job)
        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertAlmostEqual(path.poses[-1].pose.position.x, 4.95)

    def test_blocked_request_keeps_priority_while_old_search_finishes(self):
        self.plan_horizontal()
        self.node.replan_period_s = 0.5
        started, release = Event(), Event()
        self.addCleanup(release.set)

        def slow_search(snapshot, settings, position, yaw, goal, goal_yaw, cancel,
                        *, allow_escape):
            started.set()
            if not release.wait(3):
                raise RuntimeError('Test did not release search.')
            return find_route(snapshot, settings, position, yaw, goal, goal_yaw,
                              lambda: False, allow_escape=allow_escape)

        with patch('luna_guidance.path_planner_node.find_route', side_effect=slow_search):
            self.set_pose_time(0.6)
            self.node.on_timer()
            self.assertTrue(started.wait(3))
            self.assertEqual(self.node.job.trigger, 'periodic')

            self.set_pose_time(0.7)
            self.node.on_map(self.map_with_obstacle(30, 20))
            self.node._on_map_ready()
            self.assertTrue(self.node.job.cancel.is_set())
            self.assertTrue(self.node.blocked_replan_pending)
            release.set()
            self.node.job.future.result(timeout=5)
            self.node._on_search_ready()
            self.assertIsNone(self.node.job)
            self.assertTrue(self.node.blocked_replan_pending)

        self.node.on_timer()
        self.assertIsNotNone(self.node.job)
        self.assertEqual(self.node.job.trigger, 'path blocked')
        self.assertAlmostEqual(self.node.last_attempt, 0.7)
        self.assertFalse(self.node.blocked_replan_pending)
        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertAlmostEqual(path.poses[-1].pose.position.x, 4.95)

    def test_map_behind_progress_and_duplicate_map_leave_route_active(self):
        self.plan_horizontal()
        for step in range(1, 11):
            self.set_pose_time(step * 0.5, x=1.05 + step * 0.25)
            self.node.on_timer()
        publish_count = self.node.path_pub.publish.call_count
        grid = self.map_with_obstacle(18, 20)
        self.node.on_map(grid)
        self.node.on_timer()
        revision = self.node.map_version

        grid.info.map_load_time.sec = 10
        self.node.on_map(grid)
        self.node.on_timer()
        self.assertEqual(self.node.map_version, revision)
        self.assertEqual(self.node.path_pub.publish.call_count, publish_count)
        self.assertIsNone(self.node.job)

    def test_blocked_goal_holds_then_recovers_after_map_clears(self):
        self.plan_horizontal()
        self.set_pose_time(0.6)
        self.node.on_map(self.map_with_obstacle(49, 20))
        self.node._on_map_ready()
        self.assertIsNotNone(self.node.job)
        path, status = self.finish_plan()
        self.assertEqual(path.poses, [])
        self.assertEqual(status.state, PlannerStatus.FAILED)
        self.assertIn('goal in obstacle', status.message.lower())

        self.set_pose_time(1.2)
        self.node.on_map(self.map)
        self.node._on_map_ready()
        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertAlmostEqual(path.poses[-1].pose.position.x, 4.95)

    def test_deviation_replans_without_a_map_update(self):
        self.plan_horizontal()
        self.set_pose_time(0.7, x=2.25, y=2.75)
        self.node.on_timer()
        self.assertEqual(self.node.path_pub.publish.call_args.args[0].poses, [])
        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertIn('path deviation', status.message)
        self.assertAlmostEqual(path.poses[0].pose.position.x, 2.25)
        self.assertAlmostEqual(path.poses[0].pose.position.y, 2.75)

    def test_pose_correction_cannot_skip_a_blocked_leg_of_a_detour(self):
        grid = deepcopy(self.map)
        grid.info.width, grid.info.height, grid.info.resolution = 140, 100, 0.05
        grid.data = [0] * (140 * 100)
        barrier = deepcopy(grid)
        barrier_data = list(grid.data)
        for row in (40, 41):
            barrier_data[row * 140:row * 140 + 84] = [100] * 84
        barrier.data = barrier_data
        self.node.on_map(barrier)
        self.set_pose_time(0.0, x=1.1, y=1.1)
        self.goal.pose.position.x, self.goal.pose.position.y = 1.1, 3.1
        _, status = self.plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        original = self.node.route
        self.assertGreater(max(x for x, _ in original.points), 4.9)

        self.set_pose_time(0.1, x=1.5, y=1.12)
        self.node.on_map(grid)
        self.node._on_map_ready()
        self.assertIs(self.node.route, original)

        self.set_pose_time(0.2, x=1.5, y=2.66)
        blocked = deepcopy(grid)
        blocked.data[22 * 140 + 65] = 100
        self.node.on_map(blocked)
        self.node._on_map_ready()
        self.assertEqual(self.node.path_pub.publish.call_args.args[0].poses, [])
        self.assertIsNone(self.node.route)
        self.assertEqual(self.node.job.trigger, 'path blocked')

        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        start = path.poses[0].pose.position
        goal = path.poses[-1].pose.position
        self.assertAlmostEqual(start.x, 1.5)
        self.assertAlmostEqual(start.y, 2.66)
        self.assertAlmostEqual(goal.x, 1.1)
        self.assertAlmostEqual(goal.y, 3.1)

    def test_repeated_pose_does_not_advance_the_progress_window(self):
        self.plan_horizontal()
        self.set_pose_time(0.1, x=1.85)
        self.node.on_timer()
        progress = self.node.progress
        self.assertLess(progress.point[0], 1.85)
        for seconds in (0.2, 0.3, 0.4):
            self.set_pose_time(seconds, x=1.85)
            self.node.on_timer()
            self.assertEqual(self.node.progress, progress)

    def test_stale_odometry_holds_and_recovers_with_fresh_pose(self):
        self.plan_horizontal()
        self.clock_ns = 700_000_000
        self.node.on_timer()
        self.assertEqual(self.node.path_pub.publish.call_args.args[0].poses, [])
        self.assertIn('stale', self.node.status_pub.publish.call_args.args[0].message)

        self.set_pose_time(0.8, x=1.25)
        self.node.on_timer()
        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertAlmostEqual(path.poses[0].pose.position.x, 1.25)

    def test_latest_map_can_reject_completed_search(self):
        with self.pause_search() as (started, release):
            self.node.on_odom(self.odom)
            self.node.on_goal(self.goal)
            self.node.on_timer()
            self.assertTrue(started.wait(3))
            self.set_pose_time(0.1)
            self.node.on_map(self.map_with_obstacle(26, 26))
            self.node.on_timer()
            self.assertFalse(self.node.job.cancel.is_set())
            release.set()
            _, status = self.finish_plan()
            self.assertEqual(status.state, PlannerStatus.FAILED)
            self.assertIn('Map changed during search', status.message)

        self.assertFalse(any(
            call.args[0].poses for call in self.node.path_pub.publish.call_args_list
        ))
        self.set_pose_time(0.6)
        self.node.on_timer()
        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertGreater(len(path.poses), 2)

    def test_failed_old_search_retries_when_goal_is_freed(self):
        self.node.on_map(self.map_with_obstacle(40, 40))
        self.node._on_map_ready()
        with self.pause_search() as (started, release):
            self.node.on_odom(self.odom)
            self.node.on_goal(self.goal)
            self.node.on_timer()
            self.assertTrue(started.wait(3))
            self.set_pose_time(0.1)
            self.node.on_map(self.map)
            self.node._on_map_ready()
            release.set()
            path, _ = self.finish_plan()
            self.assertEqual(path.poses, [])
            self.assertEqual(self.node.pending_plan[0], 'map changed')

        self.set_pose_time(0.6)
        self.node.on_timer()
        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertAlmostEqual(path.poses[-1].pose.position.x, 4.05)

    def test_geometry_change_cancels_inflight_search(self):
        with self.pause_search() as (started, release):
            self.node.on_odom(self.odom)
            self.node.on_goal(self.goal)
            self.node.on_timer()
            self.assertTrue(started.wait(3))
            self.set_pose_time(0.1)
            self.load_coarse_map()
            self.assertTrue(self.node.job.cancel.is_set())
            release.set()
            path, _ = self.finish_plan()
            self.assertEqual(path.poses, [])

        self.set_pose_time(0.6)
        self.node.on_timer()
        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertAlmostEqual(path.poses[-1].pose.position.x, 4.05)

    def test_new_goal_cancels_inflight_search(self):
        with self.pause_search() as (started, release):
            self.node.on_odom(self.odom)
            self.node.on_goal(self.goal)
            self.node.on_timer()
            self.assertTrue(started.wait(3))
            self.goal.pose.position.x = 4.55
            self.node.on_goal(self.goal)
            self.node.on_timer()
            self.assertTrue(self.node.job.cancel.is_set())
            release.set()
            self.finish_plan()

        path, status = self.finish_plan()
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertAlmostEqual(path.poses[-1].pose.position.x, 4.55)
        published = [call.args[0] for call in self.node.path_pub.publish.call_args_list]
        self.assertEqual(sum(bool(path.poses) for path in published), 1)

    def test_ready_callback_rechecks_map_before_publishing(self):
        notified = Event()
        with patch.object(self.node._search_ready, 'trigger', side_effect=notified.set):
            self.node.on_odom(self.odom)
            self.node.on_goal(self.goal)
            self.node.on_timer()
            self.assertTrue(notified.wait(5))

        # The worker is finished but the newer map is still pending.
        self.set_pose_time(0.6)
        self.node.on_map(self.map_with_obstacle(26, 26))
        self.node._on_search_ready()
        self.assertIsNone(self.node.job)
        self.assertIsNotNone(self.node.pending_plan)
        self.assertFalse(any(
            call.args[0].poses for call in self.node.path_pub.publish.call_args_list
        ))

        self.node.on_timer()
        self.node.job.future.result(timeout=5)
        self.node._on_search_ready()
        status = self.node.status_pub.publish.call_args.args[0]
        self.assertEqual(status.state, PlannerStatus.SUCCEEDED)
        self.assertGreater(len(self.node.path_pub.publish.call_args.args[0].poses), 2)


if __name__ == '__main__':
    unittest.main()
