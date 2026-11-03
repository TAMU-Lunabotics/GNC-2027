"""ROS-free grid and obstacle-proximity tests for Regulated Pure Pursuit."""
import math
import pytest
from luna_guidance.obstacle_distance import ObstacleDistance


def make_map(width=60, height=60, res=0.1, origin=(-3, -3), yaw=0):
    grid = ObstacleDistance(occupied_threshold=50)
    grid.set_map(width, height, res, origin[0], origin[1], yaw,
                 [0] * (width * height))
    return grid


def test_clear_map_and_boundaries():
    grid = make_map()
    assert grid.clearance(0, 0, 0.6) == pytest.approx(2.4)
    assert grid.clearance(2.9, 0, 0.6) < 0
    assert grid.collides(2.5, 0, 0.6)


def test_unknown_is_blocked_and_threshold():
    grid = make_map(width=10, height=10, res=1, origin=(0, 0))
    data = [0] * 100
    data[5 * 10 + 5] = -1
    grid.set_map(10, 10, 1, 0, 0, 0, data)
    assert grid.collides(5.5, 5.5, 0.2)
    data[55] = 49
    grid.set_map(10, 10, 1, 0, 0, 0, data)
    assert not grid.collides(5.5, 5.5, 0.2)
    data[55] = 50
    grid.set_map(10, 10, 1, 0, 0, 0, data)
    assert grid.collides(5.5, 5.5, 0.2)


def test_partial_updates_and_clear_again():
    grid = make_map()
    assert grid.apply_update(29, 29, 1, 1, [100])
    assert grid.collides(0, 0, 0.2)
    assert grid.apply_update(29, 29, 1, 1, [0])
    assert not grid.collides(0, 0, 0.2)


def test_reject_out_of_bounds_update():
    grid = make_map()
    with pytest.raises(ValueError):
        grid.apply_update(59, 59, 2, 2, [100] * 4)


def test_map_not_yet_ready():
    grid = ObstacleDistance()
    assert grid.clearance(1, 1, 0.1) is None
    assert grid.collides(1, 1, 0.1)
    assert not grid.apply_update(0, 0, 1, 1, [100])


def test_rotated_grid():
    grid = ObstacleDistance()
    data = [0] * 100
    data[1 * 10 + 2] = 100
    grid.set_map(10, 10, 1, 0, 0, math.pi / 2, data)
    assert grid.collides(-1.5, 2.5, 0.2)
    assert not grid.collides(-7.5, 7.5, 0.2)
