"""Tests for ROS-independent occupancy-grid helpers."""

import math
import unittest

from luna_guidance.grid import (
    GridGeometry,
    occupancy_to_blocked,
)


class TestGrid(unittest.TestCase):
    def test_coordinate_round_trip(self):
        geometries = [
            GridGeometry(10, 6, 0.5, 1.0, -2.0),
            GridGeometry(10, 6, 0.5, 1.0, -2.0, math.pi / 2),
        ]
        for geometry in geometries:
            for y in range(geometry.height):
                for x in range(geometry.width):
                    world = geometry.cell_to_world((x, y))
                    self.assertEqual(
                        geometry.world_to_cell(*world), (x, y)
                    )

    def test_rotated_coordinates(self):
        geometry = GridGeometry(3, 3, 0.5, origin_yaw=math.pi / 2)
        x, y = geometry.cell_to_world((0, 0))
        self.assertAlmostEqual(x, -0.25)
        self.assertAlmostEqual(y, 0.25)

    def test_outside_coordinates_are_not_clamped(self):
        geometry = GridGeometry(3, 3, 1.0)
        self.assertEqual(geometry.world_to_cell(-0.1, 0.5), (-1, 0))
        self.assertEqual(geometry.world_to_cell(3.0, 0.5), (3, 0))
        with self.assertRaises(ValueError):
            geometry.cell_to_world((-1, 0))

    def test_occupancy_conversion(self):
        geometry = GridGeometry(3, 2, 1.0)
        data = [0, 49, 50, 100, -1, 0]
        self.assertEqual(
            occupancy_to_blocked(data, geometry),
            [[False, False, True], [True, True, False]],
        )
        self.assertEqual(
            occupancy_to_blocked(data, geometry, unknown_is_blocked=False),
            [[False, False, True], [True, False, False]],
        )

    def test_invalid_occupancy(self):
        geometry = GridGeometry(2, 1, 1.0)
        for data in ([0], [0, 101], [0, -2], [0, 0.5]):
            with self.subTest(data=data):
                with self.assertRaises(ValueError):
                    occupancy_to_blocked(data, geometry)


if __name__ == "__main__":
    unittest.main()
