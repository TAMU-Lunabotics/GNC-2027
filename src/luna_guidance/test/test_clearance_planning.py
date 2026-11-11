"""Tests for Theta* connections using circular clearance."""

from luna_guidance.clearance import CircularClearance
from luna_guidance.grid import GridGeometry
from luna_guidance.heading_clearance import HeadingClearance
from luna_guidance.theta_star import theta_star


def test_valid_centers_with_unsafe_diagonal():
    """Reject a diagonal that passes too close to an obstacle corner."""
    geometry = GridGeometry(10, 10, 1.0)
    blocked = [[False] * 10 for _ in range(10)]
    blocked[4][4] = True

    checker = CircularClearance(blocked, geometry, 0.4, 0.0)
    grid = checker.center_blocked_grid()
    start = (3, 4)
    goal = (4, 3)

    # Both centers are 0.5 m from the occupied square.
    assert not grid[start[1]][start[0]]
    assert not grid[goal[1]][goal[0]]

    # Their diagonal passes only sqrt(0.125) ~= 0.354 m
    # from the obstacle corner, less than the 0.4 m radius.
    assert not checker.cell_segment_is_free(start, goal)

    path = theta_star(
        grid,
        start,
        goal,
        segment_is_free=checker.cell_segment_is_free,
    )

    assert path
    assert path[0] == start
    assert path[-1] == goal
    assert len(path) >= 3

    for a, b in zip(path, path[1:]):
        assert checker.cell_segment_is_free(a, b)


def test_custom_checker_controls_neighbor_moves():
    """Ensure Theta* cannot bypass rejected neighboring connections."""
    grid = [[False] * 5 for _ in range(5)]
    checked = []

    def reject_connection(a, b):
        checked.append((a, b))
        return False

    path = theta_star(
        grid, (1, 1), (3, 3),
        segment_is_free=reject_connection,
    )

    assert checked
    assert path == []


def test_custom_checker_controls_long_shortcuts():
    """Ensure parent shortcuts also obey the supplied checker."""
    grid = [[False] * 7 for _ in range(7)]
    rejected_shortcuts = []

    def adjacent_only(a, b):
        allowed = max(abs(a[0] - b[0]), abs(a[1] - b[1])) <= 1
        if not allowed:
            rejected_shortcuts.append((a, b))
        return allowed

    path = theta_star(
        grid, (1, 1), (5, 5),
        segment_is_free=adjacent_only,
    )

    assert path
    assert rejected_shortcuts
    assert path[0] == (1, 1)
    assert path[-1] == (5, 5)

    for a, b in zip(path, path[1:]):
        assert max(abs(a[0] - b[0]), abs(a[1] - b[1])) <= 1


def test_occupied_runs_preserve_contact_and_free_gap():
    """Keep exact run edges and the free cell between two occupied runs."""
    geometry = GridGeometry(12, 10, 0.1)
    blocked = [[False] * 12 for _ in range(10)]
    blocked[4][2:5] = [True] * 3
    blocked[4][6:9] = [True] * 3

    circle = CircularClearance(blocked, geometry, 0.025, 0.0)
    assert not circle.segment_is_free((0.25, 0.375), (0.85, 0.375))
    assert circle.segment_is_free((0.25, 0.3749), (0.85, 0.3749))
    assert circle.segment_is_free((0.55, 0.2), (0.55, 0.8))

    rectangle = HeadingClearance(blocked, geometry, 0.16, 0.06, 0.0, 0.0)
    assert rectangle.straight_is_free((0.55, 0.2), (0.55, 0.8))
    assert not rectangle.straight_is_free((0.525, 0.2), (0.525, 0.8))


def test_center_grid_matches_point_clearance_on_rotated_map():
    """Preserve the point check at map edges and near occupied row spans."""
    geometry = GridGeometry(16, 13, 0.1, 1.7, -0.3, 0.57)
    blocked = [[False] * geometry.width for _ in range(geometry.height)]
    blocked[3][2:6] = [True] * 4
    blocked[7][8:11] = [True] * 3
    blocked[10][13] = True
    checker = CircularClearance(blocked, geometry, 0.13, 0.02)

    expected = [
        [checker.point_reason(geometry.cell_to_world((x, y))) is not None
         for x in range(geometry.width)]
        for y in range(geometry.height)
    ]
    assert checker.center_blocked_grid() == expected
