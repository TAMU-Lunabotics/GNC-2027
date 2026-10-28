"""Direct callback tests with mocked publishers."""

from copy import deepcopy
import math
import unittest
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import PoseStamped
from luna_interfaces.msg import PlannerStatus
from nav_msgs.msg import OccupancyGrid, Odometry

from luna_guidance.path_planner_node import PathPlannerNode


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

        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.width = 60
        self.map.info.height = 60
        self.map.info.resolution = 0.1
        self.map.info.origin.orientation.w = 1.0
        self.map.data = [0] * 3600
        self.node.on_map(self.map)

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
        path = self.node.path_pub.publish.call_args.args[0]
        status = self.node.status_pub.publish.call_args.args[0]
        return path, status

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

    def test_exact_start_rejected_even_when_cell_center_is_free(self):
        # Copy rather than mutate the map retained by the node.
        grid = deepcopy(self.map)
        grid.data[30 * 60 + 30] = 100
        self.node.on_map(grid)

        # Obstacle begins at x=3.0 m. Exact start gap is about 0.81 m,
        # below 0.815 m, while its cell-center gap is about 0.85 m.
        self.odom.pose.pose.position.x = 2.19
        self.odom.pose.pose.position.y = 3.05
        cell = self.node.geometry.world_to_cell(2.19, 3.05)
        self.assertFalse(self.node.blocked[cell[1]][cell[0]])

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
        self.assertTrue(self.node.blocked[2][0])

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
        checker = self.node.clearance_checker
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


if __name__ == '__main__':
    unittest.main()
