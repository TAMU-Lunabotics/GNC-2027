"""Occupancy-to-path regression tests for the runtime clearance methods."""

import math
import unittest

from luna_guidance.clearance import CircularClearance
from luna_guidance.grid import GridGeometry, occupancy_to_blocked
from luna_guidance.heading_clearance import CircleFirstHeadingClearance
from luna_guidance.heading_theta_star import heading_theta_star
from luna_guidance.theta_star import theta_star


class TestPlannerPipeline(unittest.TestCase):
    def make_wall(self, geometry, column, height):
        data = [0] * (geometry.width * geometry.height)
        for y in range(height):
            data[y * geometry.width + column] = 100
        return occupancy_to_blocked(data, geometry)

    def circular_path(self, geometry, blocked, start, goal, radius, margin):
        checker = CircularClearance(blocked, geometry, radius, margin)
        grid = checker.center_blocked_grid()
        cells = theta_star(
            grid, geometry.world_to_cell(*start),
            geometry.world_to_cell(*goal),
            segment_is_free=checker.cell_segment_is_free,
        )
        path = [geometry.cell_to_world(cell) for cell in cells]
        for a, b in zip(path, path[1:]):
            self.assertTrue(checker.segment_is_free(a, b))
        return path

    def assert_crosses_above(self, path, wall_x, minimum_y):
        # Necessary clearance condition independent of the collision checker.
        crossings = []
        for (ax, ay), (bx, by) in zip(path, path[1:]):
            if ax != bx and min(ax, bx) <= wall_x <= max(ax, bx):
                fraction = (wall_x - ax) / (bx - ax)
                crossings.append(ay + fraction * (by - ay))
        self.assertTrue(crossings)
        for y in crossings:
            self.assertGreater(y, minimum_y)

    def test_route_around_wall(self):
        geometry = GridGeometry(40, 40, 0.1)
        blocked = self.make_wall(geometry, column=20, height=26)
        start, goal = (1.05, 1.05), (3.05, 1.05)
        path = self.circular_path(geometry, blocked, start, goal, 0.15, 0.05)
        self.assertGreaterEqual(len(path), 3)
        for actual, expected in zip((path[0], path[-1]), (start, goal)):
            self.assertAlmostEqual(math.dist(actual, expected), 0.0)
        self.assert_crosses_above(path, 2.05, 2.6 + 0.15 + 0.05)

    def test_full_wall_has_no_route(self):
        geometry = GridGeometry(40, 40, 0.1)
        blocked = self.make_wall(geometry, column=20, height=40)
        self.assertEqual(self.circular_path(
            geometry, blocked, (1.05, 1.05), (3.05, 1.05), 0.15, 0.05
        ), [])

    def test_measured_rover_circular_detour(self):
        geometry = GridGeometry(60, 50, 0.1)
        blocked = self.make_wall(geometry, column=30, height=25)
        path = self.circular_path(
            geometry, blocked, (1.05, 1.05), (5.05, 1.05), 0.715, 0.10
        )
        self.assertGreaterEqual(len(path), 3)
        self.assert_crosses_above(path, 3.05, 2.5 + 0.715 + 0.10)

    def test_hybrid_detour_segments_turns_and_cost(self):
        geometry = GridGeometry(40, 40, 0.25)
        data = [0] * (geometry.width * geometry.height)
        for y in range(16, 24):
            for x in range(18, 22):
                data[y * geometry.width + x] = 100
        checker = CircleFirstHeadingClearance(
            occupancy_to_blocked(data, geometry), geometry,
            1.2319, 0.7239, 0.10, 5.0, 0.715,
        )
        start, goal = (2.0, 5.0), (8.0, 5.0)
        self.assertFalse(checker.straight_is_free(start, goal))
        result = heading_theta_star(
            checker, start, goal, 0.0, merge_turnable=True
        )
        self.assertEqual(result['status'], 'PATH_FOUND')
        path = result['path']
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        self.assertGreaterEqual(len(path), 3)
        arrival_yaw = 0.0
        for a, b in zip(path, path[1:]):
            self.assertTrue(checker.straight_is_free(a, b))
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            turn = abs(math.atan2(
                math.sin(yaw - arrival_yaw), math.cos(yaw - arrival_yaw)
            ))
            if turn > 1e-9:
                self.assertTrue(checker.turn_is_free(a))
            arrival_yaw = yaw
        self.assertAlmostEqual(
            result['cost'],
            sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        )
        self.assertGreater(result['cost'], math.dist(start, goal))


if __name__ == '__main__':
    unittest.main()
