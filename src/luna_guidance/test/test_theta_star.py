"""Standalone tests for the Theta* planning core."""

import math
import unittest

from luna_guidance.theta_star import (
    distance,
    line_of_sight,
    simplify_path,
    theta_star,
)


class TestThetaStar(unittest.TestCase):
    def test_open_map(self):
        grid = [[False] * 7 for _ in range(7)]
        path = theta_star(grid, (0, 0), (6, 6))
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (6, 6))
        length = sum(distance(a, b) for a, b in zip(path, path[1:]))
        self.assertAlmostEqual(length, math.hypot(6, 6))
        self.assertEqual(simplify_path(path), [(0, 0), (6, 6)])

    def test_wall_detour_and_closed_gap(self):
        grid = [[False] * 7 for _ in range(7)]
        for y in range(6):
            grid[y][3] = True

        path = theta_star(grid, (1, 1), (5, 1))
        self.assertTrue(path)
        self.assertEqual(path[0], (1, 1))
        self.assertEqual(path[-1], (5, 1))

        # Independently check where each segment crosses the wall strip.
        for a, b in zip(path, path[1:]):
            if a[0] == b[0]:
                self.assertNotEqual(a[0], 3)
                continue
            for wall_x in (2.5, 3.0, 3.5):
                t = (wall_x - a[0]) / (b[0] - a[0])
                if 0 <= t <= 1:
                    crossing_y = a[1] + t * (b[1] - a[1])
                    self.assertGreater(crossing_y, 5.5)

        grid[6][3] = True
        self.assertEqual(theta_star(grid, (1, 1), (5, 1)), [])

    def test_corner_cutting(self):
        grid = [[False, True], [True, False]]
        self.assertFalse(line_of_sight(grid, (0, 0), (1, 1)))
        self.assertEqual(theta_star(grid, (0, 0), (1, 1)), [])

    def test_shallow_segment_obstacle(self):
        grid = [[False] * 5 for _ in range(3)]
        # The segment from (0, 0) to (4, 2) crosses cell (1, 0).
        grid[0][1] = True
        self.assertFalse(line_of_sight(grid, (0, 0), (4, 2)))
        self.assertFalse(line_of_sight(grid, (4, 2), (0, 0)))

    def test_invalid_endpoints(self):
        grid = [[False, True], [False, False]]
        for start, goal in [
            ((1, 0), (1, 1)),
            ((0, 0), (1, 0)),
            ((-1, 0), (1, 1)),
            ((0, 0), (2, 1)),
        ]:
            with self.subTest(start=start, goal=goal):
                with self.assertRaises(ValueError):
                    theta_star(grid, start, goal)

    def test_same_start_and_goal(self):
        self.assertEqual(theta_star([[False]], (0, 0), (0, 0)), [(0, 0)])

    def test_invalid_grid(self):
        for grid in ([], [[]], [[False], [False, False]]):
            with self.subTest(grid=grid):
                with self.assertRaises(ValueError):
                    theta_star(grid, (0, 0), (0, 0))

    def test_simplification(self):
        for path in (
            [],
            [(1, 1)],
            [(0, 0), (1, 1)],
            [(0, 0), (1, 0), (1, 1)],
            [(0, 0), (2, 0), (1, 0)],
        ):
            self.assertEqual(simplify_path(path), path)


if __name__ == "__main__":
    unittest.main()
