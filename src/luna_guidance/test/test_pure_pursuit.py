"""Run from src/luna_guidance:  pytest test/"""
import math
import random

from luna_guidance.pure_pursuit import (FOLLOWING, GOAL_REACHED, IDLE, STOPPED, Params,
                                        PurePursuit, wrap_angle)

DT = 0.05  # 20 Hz


def step_robot(x, y, yaw, v, w, dt):
    mid = yaw + 0.5 * w * dt
    return x + v * math.cos(mid) * dt, y + v * math.sin(mid) * dt, wrap_angle(yaw + w * dt)


def run(controller, pose, seconds=60.0, pose_noise=None, seed=0):
    rng = random.Random(seed)
    x, y, yaw = pose
    history = []
    for _ in range(int(seconds / DT)):
        mx, my, myaw = x, y, yaw
        if pose_noise:
            mx += rng.gauss(0, pose_noise[0])
            my += rng.gauss(0, pose_noise[0])
            myaw += rng.gauss(0, pose_noise[1])
        v, w = controller.get_control(mx, my, myaw, DT)
        x, y, yaw = step_robot(x, y, yaw, v, w, DT)
        history.append((x, y, yaw, v, w, controller.state))
        if controller.state in (GOAL_REACHED, STOPPED):
            break
    return (x, y, yaw), history


def line(x0, y0, x1, y1, spacing=0.05):
    n = max(int(math.hypot(x1 - x0, y1 - y0) / spacing), 1)
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def test_no_path_is_idle_and_zero():
    c = PurePursuit()
    assert c.get_control(0, 0, 0, DT) == (0.0, 0.0)
    assert c.state == IDLE


def test_empty_path_stops_a_moving_robot():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0))
    for _ in range(20):
        c.get_control(0, 0, 0, DT)
    c.set_path([])
    assert c.get_control(0, 0, 0, DT) == (0.0, 0.0)
    assert c.state == IDLE


def test_straight_path_aligned_drives_straight():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0))
    for _ in range(10):
        v, w = c.get_control(0, 0, 0, DT)
    assert v > 0.0
    assert abs(w) < 1e-9
    assert c.state == FOLLOWING


def test_robot_right_of_path_turns_left():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0))
    for _ in range(10):
        v, w = c.get_control(0, -0.2, 0, DT)
    assert v > 0.0 and w > 0.0


def test_robot_left_of_path_turns_right():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0))
    for _ in range(10):
        v, w = c.get_control(0, 0.2, 0, DT)
    assert v > 0.0 and w < 0.0


def test_path_behind_rotates_in_place():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0))
    for _ in range(10):
        v, w = c.get_control(1.0, 0.0, math.pi, DT)
    assert v == 0.0
    assert abs(w) > 0.0
    assert c.reason == 'rotating in place'


def test_limits_are_respected():
    p = Params()
    c = PurePursuit(p)
    c.set_path(line(0, 0, 3, 0))
    last_v = last_w = 0.0
    for _ in range(200):
        v, w = c.get_control(0, -0.4, 1.2, DT)
        assert 0.0 <= v <= p.max_linear_speed + 1e-9
        assert abs(w) <= p.max_angular_speed + 1e-9
        assert abs(v - last_v) <= p.max_linear_accel * DT + 1e-9
        assert abs(w - last_w) <= p.max_angular_accel * DT + 1e-9
        last_v, last_w = v, w


def test_far_from_path_stops_and_stays_stopped():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0))
    assert c.get_control(0.0, 2.0, 0.0, DT) == (0.0, 0.0)
    assert c.state == STOPPED
    assert 'cross-track' in c.reason
    assert c.get_control(0.0, 0.0, 0.0, DT) == (0.0, 0.0)
    assert c.state == STOPPED
    c.set_path(line(0, 0, 3, 0))
    c.get_control(0.0, 0.0, 0.0, DT)
    assert c.state == FOLLOWING


def test_stop_is_not_latched():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0))
    assert c.stop('no localization') == (0.0, 0.0)
    assert c.state == STOPPED and c.reason == 'no localization'
    c.get_control(0.0, 0.0, 0.0, DT)
    assert c.state == FOLLOWING


def test_stop_does_not_overwrite_goal_reached():
    c = PurePursuit()
    c.set_path(line(0, 0, 1, 0))
    c.get_control(0.98, 0.0, 0.0, DT)
    assert c.state == GOAL_REACHED
    c.stop('no localization')
    assert c.state == GOAL_REACHED


def test_goal_reached_latches():
    c = PurePursuit()
    c.set_path(line(0, 0, 1, 0))
    assert c.get_control(0.98, 0.0, 0.0, DT) == (0.0, 0.0)
    assert c.state == GOAL_REACHED
    c.get_control(0.5, 0.0, 0.0, DT)
    assert c.state == GOAL_REACHED


def test_closed_loop_straight_path_reaches_goal():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0))
    (x, y, _), history = run(c, (0.0, 0.1, 0.1))
    assert c.state == GOAL_REACHED
    assert math.hypot(x - 3, y) <= 0.10
    assert max(abs(h[3]) for h in history) <= 0.5 + 1e-9


def test_closed_loop_does_not_overshoot_goal():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0))
    _, history = run(c, (0.0, 0.0, 0.0))
    assert max(h[0] for h in history) < 3.0 + 0.15


def test_closed_loop_corner_reaches_goal():
    c = PurePursuit()
    c.set_path(line(0, 0, 2, 0)[:-1] + line(2, 0, 2, 2))
    (x, y, _), _ = run(c, (0.0, 0.0, 0.0))
    assert c.state == GOAL_REACHED
    assert math.hypot(x - 2, y - 2) <= 0.10


def test_closed_loop_starts_facing_away():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0))
    (x, y, _), history = run(c, (0.0, 0.0, math.pi))
    assert c.state == GOAL_REACHED
    assert math.hypot(x - 3, y) <= 0.10
    assert history[0][3] == 0.0


def test_closed_loop_u_turn_does_not_skip_to_the_other_branch():
    path = line(0, 0, 2, 0)[:-1] + line(2, 0, 2, 0.4)[:-1] + line(2, 0.4, 0, 0.4)
    c = PurePursuit()
    c.set_path(path)
    (x, y, _), history = run(c, (0.0, 0.0, 0.0), seconds=90)
    assert c.state == GOAL_REACHED
    assert max(h[0] for h in history) > 1.8


def test_closed_loop_final_heading_is_aligned():
    c = PurePursuit()
    c.set_path(line(0, 0, 2, 0), goal_yaw=math.pi / 2)
    (x, y, yaw), _ = run(c, (0.0, 0.0, 0.0))
    assert c.state == GOAL_REACHED
    assert abs(wrap_angle(yaw - math.pi / 2)) <= 0.10


def test_closed_loop_with_noisy_localization():
    c = PurePursuit()
    c.set_path(line(0, 0, 3, 0)[:-1] + line(3, 0, 3, 2))
    (x, y, _), history = run(c, (0.0, 0.0, 0.0), pose_noise=(0.01, 0.005), seed=1)
    assert c.state == GOAL_REACHED
    assert math.hypot(x - 3, y - 2) <= 0.15
    assert max(abs(h[4]) for h in history) <= 1.0 + 1e-9


def test_closed_loop_tracks_path_closely():
    c = PurePursuit()
    c.set_path(line(0, 0, 4, 0))
    _, history = run(c, (0.0, 0.0, 0.0))
    assert max(abs(h[1]) for h in history) < 0.05


def test_single_point_path():
    c = PurePursuit()
    c.set_path([(1.0, 0.0)])
    (x, y, _), _ = run(c, (0.0, 0.0, 0.0))
    assert c.state == GOAL_REACHED
    assert math.hypot(x - 1.0, y) <= 0.10
