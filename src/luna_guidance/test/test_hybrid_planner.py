"""Regression checks for hybrid geometry and planner-wrapper behavior."""

import math
import unittest

from luna_guidance.grid import GridGeometry
from luna_guidance.heading_clearance import CircleFirstHeadingClearance
from luna_guidance.heading_theta_star import heading_theta_star
from luna_guidance.route_search import PlanningMap, SearchSettings, find_route


def make_checker(corridor=False):
    geometry = GridGeometry(80, 60, 0.1)
    blocked = [[False] * geometry.width for _ in range(geometry.height)]
    if corridor:
        # Free corridor y=(2.0, 3.2): rectangle fits; enclosing circle does not.
        for y in range(geometry.height):
            if y < 20 or y >= 32:
                blocked[y] = [True] * geometry.width

    return CircleFirstHeadingClearance(
        blocked, geometry,
        length_m=1.2319,
        width_m=0.7239,
        margin_m=0.1,
        heading_error_deg=5.0,
        turn_radius_m=0.715,
    )


def plan_hybrid(checker, goal, goal_yaw, start=(2.0, 2.6), yaw=0.0):
    geometry = checker.geometry
    blocked = [[False] * geometry.width for _ in range(geometry.height)]
    for (x, y), _ in checker.circle.obstacles:
        blocked[y][x] = True
    settings = SearchSettings('hybrid', 0.715, 0.1, 1.2319, 0.7239, 5.0,
                              0.0, 20000, True, False, 0.3)
    snapshot = PlanningMap(geometry, blocked, checker)
    return list(find_route(snapshot, settings, start, yaw, goal, goal_yaw,
                           lambda: False).points)


class TestHybridPlanner(unittest.TestCase):
    def test_open_space_exact_endpoints(self):
        checker = make_checker()
        start, goal = (2.03, 2.07), (5.13, 3.17)
        yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
        points = plan_hybrid(checker, goal, yaw, start, yaw)
        self.assertEqual(points, [start, goal])

    def test_rectangle_recovers_circle_rejected_passage(self):
        checker = make_checker(corridor=True)
        start, goal = (2.0, 2.6), (6.0, 2.6)
        self.assertFalse(checker.circle.segment_is_free(start, goal))
        self.assertTrue(checker.straight_is_free(start, goal))
        self.assertFalse(checker.turn_is_free(start))
        points = plan_hybrid(checker, goal, 0.0)
        self.assertEqual(points, [start, goal])

    def test_narrow_passage_rejects_final_rotation(self):
        checker = make_checker(corridor=True)
        with self.assertRaisesRegex(ValueError, 'GOAL_TURN_REJECTED'):
            plan_hybrid(checker, (6.0, 2.6), math.pi / 2)

    def test_same_position_checks_final_rotation(self):
        checker = make_checker(corridor=True)
        start = (2.0, 2.6)
        self.assertEqual(
            plan_hybrid(checker, start, 0.0, start),
            [start],
        )
        with self.assertRaisesRegex(ValueError, 'GOAL_TURN_REJECTED'):
            plan_hybrid(checker, start, math.pi / 2, start)

    def test_open_space_final_rotation_allowed(self):
        checker = make_checker()
        points = plan_hybrid(checker, (6.0, 2.6), math.pi / 2)
        self.assertEqual(points[-1], (6.0, 2.6))

    def test_obstacle_between_endpoints_is_rejected(self):
        geometry = GridGeometry(80, 60, 0.1)
        blocked = [[False] * 80 for _ in range(60)]
        for row in blocked:
            row[40] = True
        checker = CircleFirstHeadingClearance(
            blocked, geometry, 1.2319, 0.7239, 0.1, 5.0, 0.715
        )
        self.assertFalse(checker.straight_is_free((2, 3), (6, 3)))

    def test_expansion_limit_is_not_reported_as_no_route(self):
        geometry = GridGeometry(80, 60, 0.1)
        blocked = [[False] * 80 for _ in range(60)]
        for row in blocked:
            row[40] = True
        checker = CircleFirstHeadingClearance(
            blocked, geometry, 1.2319, 0.7239, 0.1, 5.0, 0.715
        )
        result = heading_theta_star(
            checker, (2, 3), (6, 3), 0.0,
            max_expansions=1, merge_turnable=True,
        )
        self.assertEqual(result['status'], 'EXPANSION_LIMIT')

    def test_missing_start_heading_rejected(self):
        with self.assertRaisesRegex(ValueError, 'starting orientation'):
            plan_hybrid(make_checker(), (6, 3), 0.0, yaw=None)


if __name__ == '__main__':
    unittest.main()
