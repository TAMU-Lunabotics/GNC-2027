"""Check the geometry that must remain valid after a map change."""

import math

from luna_guidance.clearance import CircularClearance
from luna_guidance.grid import GridGeometry
from luna_guidance.heading_clearance import CircleFirstHeadingClearance
from luna_guidance.path_check import path_rejection, project_path, remaining_path
from luna_guidance.route_search import PlanningMap, SearchSettings, find_route


def test_remaining_segment_ignores_an_obstacle_that_has_been_passed():
    geometry = GridGeometry(60, 40, 0.1)
    blocked = [[False] * 60 for _ in range(40)]
    blocked[20][20] = True
    checker = CircularClearance(blocked, geometry, 0.3, 0.1)
    points = [(1.0, 2.05), (5.0, 2.05)]

    ahead = project_path(points, (1.4, 2.05))
    behind = project_path(points, (3.0, 2.05), ahead)
    assert path_rejection(checker, remaining_path(points, ahead), 0.0)
    assert path_rejection(checker, remaining_path(points, behind), 0.0) is None


def test_hybrid_path_can_be_blocked_at_a_turn_with_both_straights_clear():
    geometry = GridGeometry(120, 120, 0.05)
    blocked = [[False] * 120 for _ in range(120)]
    blocked[28][91] = True
    checker = CircleFirstHeadingClearance(blocked, geometry, 1.2319, 0.7239, 0.1, 5.0)
    points = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0)]

    assert checker.straight_is_free(*points[:2])
    assert checker.straight_is_free(*points[1:])
    assert 'stationary turn' in path_rejection(checker, points, math.pi / 2, 0.0)


def test_start_escape_checks_the_physical_footprint_and_restores_margin():
    geometry = GridGeometry(60, 40, 0.1)
    blocked = [[False] * 60 for _ in range(40)]
    blocked[19][20] = True
    settings = SearchSettings('circle', 0.715, 0.1, 1.2319, 0.7239, 5.0,
                              0.0, 20000, True, True, 0.3)
    checker = settings.checker(blocked, geometry)
    snapshot = PlanningMap(geometry, blocked, checker)
    start, goal = (1.25, 1.95), (1.15, 3.05)

    assert checker.point_reason(start) is not None
    route = find_route(snapshot, settings, start, None, goal, 0.0, lambda: False,
                       allow_escape=True)
    assert route.escape and route.points[0] == start
    assert math.dist(start, route.points[1]) <= 0.3
    physical = settings.checker(blocked, geometry, escape=True)
    assert physical.segment_is_free(*route.points[:2])
    assert path_rejection(checker, route.points[1:], 0.0) is None

    blocked[19][12] = True
    colliding = PlanningMap(geometry, blocked, settings.checker(blocked, geometry))
    try:
        find_route(colliding, settings, start, None, goal, 0.0, lambda: False,
                   allow_escape=True)
    except ValueError as exc:
        assert 'Start in collision' in str(exc)
    else:
        raise AssertionError('Physical footprint overlap must prevent escape.')
