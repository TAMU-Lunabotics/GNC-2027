"""ROS-free Regulated Pure Pursuit tests. Run from package with pytest test/."""
import math
import pytest
from luna_guidance.obstacle_distance import ObstacleDistance
from luna_guidance.regulated_pure_pursuit import (
    RegulatedParams, RegulatedPurePursuit, IDLE, FOLLOWING, STOPPED, GOAL_REACHED,
)

DT = 0.05


def grid():
    g = ObstacleDistance()
    g.set_map(200, 200, 0.1, -10, -10, 0, [0] * 40000)
    return g


def controller(path=None, params=None, obstacles=None):
    g = obstacles if obstacles is not None else grid()
    c = RegulatedPurePursuit(params or RegulatedParams(), g)
    if path is not None:
        c.set_path(path)
    return c


def settled(c, x, y, yaw=0, ticks=40):
    return [c.get_control(x, y, yaw, DT) for _ in range(ticks)][-1]


def test_no_path_and_empty_path():
    c = controller()
    assert c.get_control(0, 0, 0, DT) == (0, 0)
    assert c.state == IDLE
    c.set_path([(0, 0), (4, 0)])
    settled(c, 0, 0)
    assert c._v > 0
    c.set_path([])
    assert c._v == 0 and c._w == 0
    assert c.get_control(0, 0, 0, DT) == (0, 0)


def test_straight_tracking_and_speed_limits():
    c = controller([(0, 0), (4, 0)])
    v, w = settled(c, 0, 0)
    assert v == pytest.approx(c.p.cruise_speed)
    assert w == pytest.approx(0.0)
    assert c.state == FOLLOWING


def test_lookahead_interpolates_circle():
    c = controller([(0, 0), (1, 0), (2, 0)])
    c.get_control(0, 0, 0, DT)
    assert c.lookahead_point == pytest.approx((0.3, 0))
    settled(c, 0, 0)
    assert c.lookahead_point == pytest.approx((0.45, 0), abs=0.01)


def test_replan_preserves_velocity_and_resets_progress():
    c = controller([(0, 0), (4, 0)])
    settled(c, 0, 0)
    v_before = c._v
    c._idx = 5  # stand-in for progress on a longer path
    c.set_path([(0, 0), (0, 4)])
    assert c._idx == 0
    assert c._v == v_before
    v, w = c.get_control(0, 0, math.pi / 2, DT)
    assert v > 0 and c.state == FOLLOWING


def test_curvature_regulates_speed():
    straight = controller([(0, 0), (4, 0)])
    curved = controller([(0, 0), (0.5, 0.3), (2, 1)])
    v_straight, _ = settled(straight, 0, 0)
    v_curved, _ = settled(curved, 0, 0)
    assert v_curved < v_straight


def test_obstacle_proximity_regulates_speed():
    clear = controller([(0, 0), (4, 0)])
    g = grid()
    # Closest occupied cell begins near y=0.9; edge clearance ~0.3 m.
    col = int((0 - (-10)) / 0.1)
    row = int((0.9 - (-10)) / 0.1)
    g.apply_update(col, row, 1, 1, [100])
    near = controller([(0, 0), (4, 0)], obstacles=g)
    v_clear, _ = settled(clear, 0, 0)
    v_near, _ = settled(near, 0, 0)
    assert near.state == FOLLOWING
    assert 0 < v_near < v_clear


def test_collision_stops_with_reason():
    g = grid()
    col = int((0.5 - (-10)) / 0.1)
    row = int((0 - (-10)) / 0.1)
    g.apply_update(col, row, 1, 1, [100])
    c = controller([(0, 0), (4, 0)], obstacles=g)
    assert c.get_control(0, 0, 0, DT) == (0, 0)
    assert c.state == STOPPED and c.reason == 'collision ahead'
    assert c.collision_arc


def test_map_update_detected_next_tick():
    g = grid()
    c = controller([(0, 0), (4, 0)], obstacles=g)
    c.get_control(0, 0, 0, DT)
    col = int((0.5 - (-10)) / 0.1)
    row = int((0 - (-10)) / 0.1)
    g.apply_update(col, row, 1, 1, [100])
    assert c.get_control(0, 0, 0, DT) == (0, 0)
    assert c.state == STOPPED


def test_approach_slows_near_goal():
    far = controller([(0, 0), (4, 0)])
    near = controller([(0, 0), (1, 0)])
    v_far, _ = settled(far, 0, 0)
    v_near, _ = settled(near, 0.7, 0)
    assert 0 < v_near < v_far


def test_no_map_stops_safely_then_resumes():
    g = ObstacleDistance()
    c = controller([(0, 0), (4, 0)], obstacles=g)
    assert c.get_control(0, 0, 0, DT) == (0, 0)
    assert c.state == STOPPED and c.reason == 'no map yet'
    g.set_map(200, 200, 0.1, -10, -10, 0, [0] * 40000)
    v, _ = c.get_control(0, 0, 0, DT)
    assert v > 0


def test_acceleration_limits():
    p = RegulatedParams()
    c = controller([(0, 0), (4, 0)], params=p)
    prev_v, prev_w = 0, 0
    for _ in range(50):
        v, w = c.get_control(0, 0, 0, DT)
        assert abs(v - prev_v) <= p.max_linear_accel * DT + 1e-8
        assert abs(w - prev_w) <= p.max_angular_accel * DT + 1e-8
        assert 0 <= v <= p.max_linear_speed
        assert abs(w) <= p.max_angular_speed
        prev_v, prev_w = v, w


def test_rotate_in_place_for_large_heading_error():
    c = controller([(0, 0), (4, 0)])
    v, w = c.get_control(0, 0, math.pi, DT)
    assert v == 0 and w != 0
    assert c.reason == 'rotating in place'


def test_goal_reached_at_end():
    c = controller([(0, 0), (1, 0)])
    v, w = c.get_control(0.98, 0, 0, DT)
    assert (v, w) == (0, 0)
    assert c.state == GOAL_REACHED


def test_collision_arc_sample_spacing():
    c = controller([(0, 0), (4, 0)])
    settled(c, 0, 0)
    assert len(c.collision_arc) > 5
    for a, b in zip(c.collision_arc, c.collision_arc[1:]):
        assert math.hypot(a[0] - b[0], a[1] - b[1]) <= c.obstacles.resolution + 1e-9
