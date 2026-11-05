"""Tests for Theta* connections using circular clearance."""

from luna_guidance.clearance import CircularClearance
from luna_guidance.grid import GridGeometry
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
