"""Check stop commands when a hold fits between controller timer ticks."""

import unittest
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import PoseStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.time import Time

from luna_guidance import pure_pursuit as pp
from luna_guidance import regulated_pure_pursuit as rpp
from luna_guidance.guidance_controller_node import GuidanceControllerNode
from luna_guidance.hybrid_path_follower import HybridPathFollower

CONTROLLER_MODES = ('pure_pursuit', 'regulated_pure_pursuit', 'hybrid')


class TestGuidanceControllerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[
            '--ros-args',
            '-r', '/cmd_vel:=/controller_test/cmd_vel',
            '-r', '/guidance/path:=/controller_test/path',
            '-r', '/guidance/status:=/controller_test/status',
            '-r', '/guidance/lookahead:=/controller_test/lookahead',
            '-r', '/guidance/collision_arc:=/controller_test/collision_arc',
        ])

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def moving_node(self, mode):
        node = GuidanceControllerNode()
        self.addCleanup(node.destroy_node)
        if mode == 'hybrid':
            node.controller_type = 'regulated_pure_pursuit'
            node.footprint_mode = 'hybrid'
            node.controller = HybridPathFollower(obstacles=node.obstacle_distance)
        elif mode == 'regulated_pure_pursuit':
            node.controller_type = mode
            node.controller = rpp.RegulatedPurePursuit(obstacles=node.obstacle_distance)
        else:
            node.controller = pp.PurePursuit()
        node.cmd_pub = Mock()
        node.status_pub = Mock()
        node.lookahead_pub = Mock()
        node.arc_pub = Mock()
        self.clock_ns = 1_000_000_000
        clock = Mock()
        clock.now.side_effect = lambda: Time(nanoseconds=self.clock_ns)
        node.get_clock = Mock(return_value=clock)

        node.on_map(self.empty_grid())

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = odom.pose.pose.position.y = 2.0
        odom.pose.pose.orientation.w = 1.0
        node.on_path(self.path([(2.0, 2.0), (5.0, 2.0)]))
        for _ in range(20):
            self.clock_ns += 50_000_000
            node.on_odom(odom)
            node.on_timer()
        self.assertGreater(node.cmd_pub.publish.call_args.args[0].twist.linear.x, 0.0)
        node.cmd_pub.reset_mock()
        return node, odom

    @staticmethod
    def empty_grid():
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 80
        grid.info.resolution = 0.1
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 6400
        return grid

    @staticmethod
    def path(points, frame='map'):
        message = Path()
        message.header.frame_id = frame
        for x, y in points:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def assert_zero(self, command):
        self.assertEqual(command.header.frame_id, 'base_link')
        self.assertEqual(command.twist.linear.x, 0.0)
        self.assertEqual(command.twist.angular.z, 0.0)

    def test_empty_path_publishes_zero_without_timer(self):
        for mode in CONTROLLER_MODES:
            with self.subTest(mode=mode):
                node, _ = self.moving_node(mode)
                self.clock_ns += 1_000_000
                node.on_path(self.path([]))
                node.cmd_pub.publish.assert_called_once()
                self.assert_zero(node.cmd_pub.publish.call_args.args[0])
                self.assertFalse(node.controller.has_path)

    def test_replacement_before_next_tick_keeps_stop_and_resets_speed(self):
        for mode in CONTROLLER_MODES:
            for hold_ms in (1, 40):
                with self.subTest(mode=mode, hold_ms=hold_ms):
                    node, odom = self.moving_node(mode)
                    last_tick = self.clock_ns
                    self.clock_ns += 1_000_000
                    node.on_path(self.path([]))
                    self.clock_ns += hold_ms * 1_000_000
                    node.on_path(self.path([(2.0, 2.0), (5.5, 2.0)]))
                    node.cmd_pub.publish.assert_called_once()
                    self.assert_zero(node.cmd_pub.publish.call_args.args[0])

                    self.clock_ns = last_tick + 50_000_000
                    node.on_odom(odom)
                    node.on_timer()
                    command = node.cmd_pub.publish.call_args.args[0]
                    self.assertGreater(command.twist.linear.x, 0.0)
                    self.assertLessEqual(
                        command.twist.linear.x,
                        node.controller.p.max_linear_accel * 0.05 + 1e-9,
                    )

    def test_wrong_frame_stops_without_timer(self):
        for mode in CONTROLLER_MODES:
            with self.subTest(mode=mode):
                node, _ = self.moving_node(mode)
                node.on_path(self.path([(2.0, 2.0), (5.0, 2.0)], frame='odom'))
                node.cmd_pub.publish.assert_called_once()
                self.assert_zero(node.cmd_pub.publish.call_args.args[0])
                self.assertFalse(node.controller.has_path)

    def test_hold_stays_zero_on_later_ticks(self):
        for mode in CONTROLLER_MODES:
            with self.subTest(mode=mode):
                node, odom = self.moving_node(mode)
                node.on_path(self.path([]))
                for _ in range(3):
                    self.clock_ns += 50_000_000
                    node.on_odom(odom)
                    node.on_timer()
                self.assertEqual(node.cmd_pub.publish.call_count, 4)
                for call in node.cmd_pub.publish.call_args_list:
                    self.assert_zero(call.args[0])

    def test_nonempty_replacement_keeps_following(self):
        for mode in CONTROLLER_MODES:
            with self.subTest(mode=mode):
                node, odom = self.moving_node(mode)
                node.on_path(self.path([(2.0, 2.0), (5.5, 2.0)]))
                node.cmd_pub.publish.assert_not_called()
                self.clock_ns += 50_000_000
                node.on_odom(odom)
                node.on_timer()
                command = node.cmd_pub.publish.call_args.args[0]
                self.assertAlmostEqual(command.twist.linear.x, node.controller.p.cruise_speed)

    def test_hybrid_keeps_early_patches_when_base_map_arrives(self):
        node = GuidanceControllerNode()
        self.addCleanup(node.destroy_node)
        node.footprint_mode = 'hybrid'
        patch = OccupancyGridUpdate()
        patch.header.frame_id = 'map'
        patch.x = patch.y = 20
        patch.width = patch.height = 1
        patch.data = [100]
        node.on_map_update(patch)
        self.assertFalse(node.obstacle_distance.ready)

        grid = self.empty_grid()
        node.on_map(grid)
        self.assertEqual(node.obstacle_distance.data[20 * 80 + 20], 100)
        node.on_map(grid)
        self.assertEqual(node.obstacle_distance.data[20 * 80 + 20], 100)
