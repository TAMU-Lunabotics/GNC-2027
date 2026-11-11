"""Drive and collision-check the path shapes returned by hybrid planning."""

import math

from luna_guidance.grid import GridGeometry
from luna_guidance.heading_clearance import HeadingClearance
from luna_guidance.hybrid_path_follower import HybridPathFollower
from luna_guidance.obstacle_distance import ObstacleDistance
from luna_guidance.pure_pursuit import GOAL_REACHED, STOPPED, wrap_angle
from luna_guidance.route_search import PlanningMap, SearchSettings, find_route


def corridor_map():
    """Leave a 1.2 m opening and space for turns at either end."""
    geometry = GridGeometry(160, 120, 0.05)
    blocked = [
        [60 <= x < 100 and (y < 40 or y >= 64) for x in range(geometry.width)]
        for y in range(geometry.height)
    ]
    obstacles = ObstacleDistance()
    obstacles.set_map(
        geometry.width, geometry.height, geometry.resolution, 0.0, 0.0, 0.0,
        [100 if cell else 0 for row in blocked for cell in row],
    )
    return geometry, blocked, obstacles


def test_planned_route_drives_through_corridor_and_turns():
    geometry, blocked, obstacles = corridor_map()
    settings = SearchSettings('hybrid', 0.715, 0.1, 1.2319, 0.7239, 5.0,
                              0.0, 20000, True, True, 0.0)
    checker = settings.checker(blocked, geometry)
    start, goal = (1.25, 2.6), (6.45, 3.65)
    goal_yaw = math.pi / 2
    points = find_route(PlanningMap(geometry, blocked, checker), settings,
                        start, 0.0, goal, goal_yaw, lambda: False).points
    assert len(points) == 3
    assert not checker.circle.segment_is_free(*points[:2])
    # The controller receives sampled poses, including each original vertex.
    sampled = [start]
    for a, b in zip(points, points[1:]):
        sampled.append(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))
        sampled.append(b)
    controller = HybridPathFollower(obstacles=obstacles)
    controller.set_path(sampled, goal_yaw)
    assert controller.path == list(points)

    truth = HeadingClearance(blocked, geometry, 1.2319, 0.7239, 0.0, 0.0)
    x, y = start
    yaw = 0.0
    last_command = (0.0, 0.0)
    turned_at = []
    dt = 0.05
    for _ in range(1600):
        v, w = controller.get_control(x, y, yaw, dt, measured_twist=last_command)
        assert controller.state != STOPPED, controller.reason
        if v == 0.0 and w != 0.0:
            assert last_command[0] == 0.0
            assert truth.turn_is_free((x, y))
            turned_at.append((x, y))
        midpoint_yaw = yaw + w * dt / 2
        x += v * math.cos(midpoint_yaw) * dt
        y += v * math.sin(midpoint_yaw) * dt
        yaw = wrap_angle(yaw + w * dt)
        assert truth.pose_is_free((x, y), yaw)
        last_command = (v, w)
        if controller.state == GOAL_REACHED:
            break

    assert controller.state == GOAL_REACHED
    assert math.dist((x, y), goal) <= controller.p.goal_tolerance_m
    assert abs(wrap_angle(yaw - goal_yaw)) <= controller.p.yaw_tolerance_rad
    assert any(math.dist(point, points[1]) <= 0.025 for point in turned_at)
    assert any(math.dist(point, goal) <= 0.025 for point in turned_at)


def test_patch_blocks_the_projected_sweep():
    _, _, obstacles = corridor_map()
    controller = HybridPathFollower(obstacles=obstacles)
    controller.set_path([(1.25, 2.6), (6.45, 2.6)])
    for _ in range(20):
        v, w = controller.get_control(1.25, 2.6, 0.0, 0.05)
    assert v > 0.0 and w == 0.0
    obstacles.apply_update(41, 52, 1, 1, [100])
    assert controller.get_control(1.25, 2.6, 0.0, 0.05) == (0.0, 0.0)
    assert controller.state == STOPPED and controller.reason == 'collision ahead'


def test_turn_is_rejected_inside_the_corridor():
    _, _, obstacles = corridor_map()
    controller = HybridPathFollower(obstacles=obstacles)
    controller.set_path([(3.5, 2.6), (6.45, 2.6)])
    assert controller.get_control(3.5, 2.6, 0.3, 0.05) == (0.0, 0.0)
    assert controller.state == STOPPED and controller.reason == 'turn clearance blocked'
    v, w = controller.get_control(3.5, 2.6, 0.0, 0.05)
    assert v > 0.0 and w == 0.0


def test_turn_waits_for_linear_motion_to_stop():
    _, _, obstacles = corridor_map()
    controller = HybridPathFollower(obstacles=obstacles)
    controller.set_path([(1.25, 2.6), (6.45, 2.6)])
    pose = (1.25, 2.6, math.pi / 2)
    assert controller.get_control(*pose, 0.05, measured_twist=(0.15, 0.0)) == (0.0, 0.0)
    v, w = controller.get_control(*pose, 0.05, measured_twist=(0.0, 0.0))
    assert v == 0.0 and w < 0.0
