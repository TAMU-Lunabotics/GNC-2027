#!/usr/bin/env python3
"""ROS 2 Humble integration + pure-geometry smoke tests for mock_tag_detector.

Run from the GNC-2027 root after sourcing the ROS 2 workspace:
 PYTHONPATH="$PWD/src/luna_sim:$PYTHONPATH" python3 -m unittest discover \
   -s tests -p 'test_mock_tag_detector_final.py' -v

Tests use a real node and ROS 2 messages; no Gazebo/RViz required.
"""
from __future__ import annotations
import math
import time
from pathlib import Path
import unittest
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from luna_interfaces.msg import TagDetectionArray
from visualization_msgs.msg import MarkerArray
from luna_sim.sensors.mock_tag_detector import (
    MockTagDetector, rotation_from_rpy, inverse_transform,
    transform_from_xyz_rpy, is_tag_detectable,
)
from visualization_msgs.msg import Marker

ROOT = Path(__file__).resolve().parents[1]
ARENA = ROOT / 'src/luna_sim/generated/artemis_arena_42.json'
GEOMETRY = ROOT / 'src/luna_description/config/robot_geometry.yaml'


def pose_at(x, y, yaw, stamp_sec=123):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = stamp_sec
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


class TestMockTagDetector(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not ARENA.is_file():
            raise RuntimeError(f'Missing generated arena: {ARENA}. Run arena_gen --seed 42.')
        if not GEOMETRY.is_file():
            raise RuntimeError(f'Missing camera geometry: {GEOMETRY}')
        rclpy.init(args=[
            '--ros-args', '-p', f'arena_json:={ARENA}',
            '-p', f'geometry_yaml:={GEOMETRY}', '-p', 'seed:=42',
        ])

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.node = MockTagDetector()

    def tearDown(self):
        self.node.destroy_node()

    def test_transform_inverse(self):
        T = transform_from_xyz_rpy([1.2, -0.4, 0.5], [0.2, -0.3, 1.1])
        np.testing.assert_allclose(T @ inverse_transform(T), np.eye(4), atol=1e-10)
        R = rotation_from_rpy(0.2, -0.3, 1.1)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)

    def test_empty_before_first_pose(self):
        self.assertIsNone(self.node.make_detection_arrays())
        self.assertIsNone(self.node.get_camera_transforms())
        self.assertEqual(len(self.node.make_sightline_markers().markers), 0)

    def test_arena_geometry_and_timer(self):
        self.assertEqual(len(self.node.tags), 4)
        self.assertEqual(set(self.node.detection_publishers), {'cam_front', 'cam_rear'})
        self.assertAlmostEqual(self.node.publish_timer.timer_period_ns / 1e9, 0.1)
        self.assertEqual(self.node.marker_publisher.topic_name, '/tags/markers')

    def test_detection_and_empty_rear_array(self):
        self.node.ground_truth_callback(pose_at(2.5, 1.25, math.pi))
        arrays = self.node.make_detection_arrays()
        self.assertEqual([d.id for d in arrays['cam_front'].detections], [0])
        self.assertEqual(len(arrays['cam_rear'].detections), 0)
        for camera, msg in arrays.items():
            self.assertEqual(msg.header.frame_id, f'{camera}_optical')
            self.assertEqual(msg.header.stamp.sec, 123)
        detected = arrays['cam_front'].detections[0]
        self.assertEqual(len(detected.pose.covariance), 36)
        self.assertGreater(detected.pose.covariance[0], 0)
        q = detected.pose.pose.orientation
        self.assertAlmostEqual(sum(v*v for v in (q.x,q.y,q.z,q.w)), 1.0, places=6)

    def test_fixed_tag_markers(self):
        markers = self.node.make_tag_markers().markers
        self.assertEqual([m.id for m in markers], [0, 1, 2, 3])
        for m in markers:
            self.assertEqual(m.header.frame_id, 'map')
            self.assertEqual(m.type, Marker.CUBE)
            self.assertGreater(m.scale.x, 0)
            self.assertGreater(m.color.a, 0)
        self.assertAlmostEqual(markers[0].pose.position.y, 1.25)

    def test_sightlines_add_and_remove(self):
        self.node.ground_truth_callback(pose_at(2.5, 1.25, math.pi))
        lines = self.node.make_sightline_markers().markers
        self.assertEqual(len(lines), 2)
        self.assertEqual([m.id for m in lines], [0, 1])
        front, rear = lines
        self.assertEqual(front.ns, 'april_tag_sightlines')
        self.assertEqual(front.type, Marker.LINE_LIST)
        self.assertEqual(front.action, Marker.ADD)
        self.assertEqual(len(front.points), 2)
        self.assertAlmostEqual(front.points[-1].x, 0.0)
        self.assertAlmostEqual(front.points[-1].y, 1.25)
        self.assertEqual(rear.action, Marker.DELETE)
        self.assertEqual(len(rear.points), 0)
        # Opposite direction: front no longer sees the left-wall tag.
        self.node.ground_truth_callback(pose_at(2.5, 1.25, 0.0))
        lines2 = self.node.make_sightline_markers().markers
        self.assertEqual(lines2[0].action, Marker.DELETE)
        self.assertEqual(len(lines2[0].points), 0)

    def test_seed_is_reproducible(self):
        second = MockTagDetector()
        try:
            position = pose_at(2.5, 1.25, math.pi)
            self.node.ground_truth_callback(position)
            second.ground_truth_callback(position)
            a = self.node.make_detection_arrays()['cam_front'].detections[0]
            b = second.make_detection_arrays()['cam_front'].detections[0]
            pa = a.pose.pose.position
            pb = b.pose.pose.position
            self.assertEqual((pa.x, pa.y, pa.z), (pb.x, pb.y, pb.z))
            self.assertEqual(list(a.pose.covariance), list(b.pose.covariance))
        finally:
            second.destroy_node()

    def test_ros_publish_transport(self):
        """Actual ROS serialization and topic transport, not just callback execution."""
        watcher = rclpy.create_node('mock_tag_detector_test_watcher')
        front_msgs, marker_msgs = [], []
        front_sub = watcher.create_subscription(
            TagDetectionArray, '/cam_front/detections', front_msgs.append, 10)
        marker_sub = watcher.create_subscription(
            MarkerArray, '/tags/markers', marker_msgs.append, 10)
        try:
            self.node.ground_truth_callback(pose_at(2.5, 1.25, math.pi))
            # Allow DDS discovery before first transmission.
            end = time.monotonic() + 5.0
            while time.monotonic() < end and (
                self.node.detection_publishers['cam_front'].get_subscription_count() == 0
                or self.node.marker_publisher.get_subscription_count() == 0
            ):
                rclpy.spin_once(watcher, timeout_sec=0.1)
            self.assertGreater(self.node.detection_publishers['cam_front'].get_subscription_count(), 0)
            self.assertGreater(self.node.marker_publisher.get_subscription_count(), 0)
            self.node.publish_detections()
            end = time.monotonic() + 5.0
            while time.monotonic() < end and (not front_msgs or not marker_msgs):
                rclpy.spin_once(watcher, timeout_sec=0.1)
            self.assertTrue(front_msgs, 'Front camera message not delivered over ROS 2')
            self.assertTrue(marker_msgs, 'MarkerArray not delivered over ROS 2')
            self.assertEqual([d.id for d in front_msgs[0].detections], [0])
            lines = [m for m in marker_msgs[0].markers if m.ns == 'april_tag_sightlines']
            self.assertEqual(len(lines), 2)
            self.assertEqual(lines[0].action, Marker.ADD)
        finally:
            watcher.destroy_subscription(front_sub)
            watcher.destroy_subscription(marker_sub)
            watcher.destroy_node()


if __name__ == '__main__':
    unittest.main(verbosity=2)
