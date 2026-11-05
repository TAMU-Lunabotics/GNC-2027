"""Visibility: the range gate on edge distance, the FOV wedge, and the noise model."""
import math

from luna_sim.sensors.mock_obstacle_detector import (
    apply_noise, edge_distance, in_fov, in_range, is_visible, noise_scale, sensor_origin,
)
import random

MIN_RANGE = 0.3
MAX_RANGE = 3.0
FOV = 90.0


def test_edge_distance_subtracts_the_radius():
    assert edge_distance(0.0, 0.0, 1.0, 0.0, 0.2) == 1.0 - 0.2


def test_sensor_inside_footprint_stays_visible():
    # Negative edge distance: sensor is inside the obstacle. Must NOT be eaten by min range.
    assert in_range(-0.5, MIN_RANGE, MAX_RANGE)
    assert in_range(0.0, MIN_RANGE, MAX_RANGE)


def test_min_range_blind_spot_hides_obstacles():
    assert not in_range(0.01, MIN_RANGE, MAX_RANGE)
    assert not in_range(0.29, MIN_RANGE, MAX_RANGE)


def test_detection_band_is_inclusive():
    assert in_range(MIN_RANGE, MIN_RANGE, MAX_RANGE)
    assert in_range(1.5, MIN_RANGE, MAX_RANGE)
    assert in_range(MAX_RANGE, MIN_RANGE, MAX_RANGE)


def test_beyond_max_range_is_hidden():
    assert not in_range(MAX_RANGE + 0.01, MIN_RANGE, MAX_RANGE)


def test_fov_wedge_is_centered_on_heading():
    def at(bearing_deg):
        a = math.radians(bearing_deg)
        return in_fov(0.0, 0.0, 0.0, 2.0 * math.cos(a), 2.0 * math.sin(a), FOV)

    assert at(0.0)
    assert at(44.0)
    assert at(-44.0)
    assert at(45.0)        # half-FOV boundary is inclusive
    assert not at(46.0)
    assert not at(90.0)
    assert not at(180.0)


def test_fov_handles_heading_wraparound():
    # Heading near +pi, obstacle just across the -pi seam: 16 deg apart, so visible.
    assert in_fov(0.0, 0.0, 3.0, 2.0 * math.cos(-3.0), 2.0 * math.sin(-3.0), FOV)


def test_fov_360_accepts_everything():
    assert in_fov(0.0, 0.0, 0.0, -2.0, 0.0, 360.0)


def test_is_visible_combines_both_gates():
    # In band, in front.
    assert is_visible(0.0, 0.0, 0.0, 1.0, 0.0, 0.2, MIN_RANGE, MAX_RANGE, FOV)
    # In band, but behind the robot.
    assert not is_visible(0.0, 0.0, 0.0, -1.0, 0.0, 0.2, MIN_RANGE, MAX_RANGE, FOV)
    # In front, but edge is inside the blind spot.
    assert not is_visible(0.0, 0.0, 0.0, 0.4, 0.0, 0.2, MIN_RANGE, MAX_RANGE, FOV)
    # Sensor inside the footprint, dead ahead.
    assert is_visible(0.0, 0.0, 0.0, 0.1, 0.0, 0.2, MIN_RANGE, MAX_RANGE, FOV)
    # A big obstacle: center is past max range, but its near edge is in band.
    assert is_visible(0.0, 0.0, 0.0, 3.2, 0.0, 0.5, MIN_RANGE, MAX_RANGE, FOV)


def test_sensor_origin_rotates_the_mount_offset():
    sx, sy = sensor_origin(1.0, 2.0, math.pi / 2.0, 0.3, 0.0)
    assert math.isclose(sx, 1.0, abs_tol=1e-9)
    assert math.isclose(sy, 2.3, abs_tol=1e-9)


def test_sensor_origin_is_identity_for_the_current_lidar_mount():
    # lidar_link xyz is [0.0, 0.0, 0.65]: no XY offset, so the sensor sits on the robot origin [2].
    # This test exists so it FAILS loudly if the mount ever moves off-center.
    for yaw in (0.0, 1.0, -2.5, math.pi):
        sx, sy = sensor_origin(1.5, -0.5, yaw, 0.0, 0.0)
        assert math.isclose(sx, 1.5, abs_tol=1e-12)
        assert math.isclose(sy, -0.5, abs_tol=1e-12)


def test_noise_scale_doubles_at_max_range():
    assert math.isclose(noise_scale(0.0, MAX_RANGE), 1.0)
    assert math.isclose(noise_scale(MAX_RANGE, MAX_RANGE), 2.0)
    assert math.isclose(noise_scale(MAX_RANGE / 2.0, MAX_RANGE), 1.5)


def test_zero_std_leaves_the_detection_exact():
    nx, ny, nr = apply_noise(random.Random(0), 2.0, 3.0, 0.2, 2.0, MAX_RANGE, 0.0, 0.0)
    assert (nx, ny, nr) == (2.0, 3.0, 0.2)


def test_same_seed_gives_the_same_detections():
    args = (1.0, 2.0, 0.2, 1.5, MAX_RANGE, 0.03, 0.02)
    assert apply_noise(random.Random(42), *args) == apply_noise(random.Random(42), *args)


def test_noise_actually_perturbs_the_detection():
    nx, ny, nr = apply_noise(random.Random(42), 1.0, 2.0, 0.2, 1.5, MAX_RANGE, 0.03, 0.02)
    assert (nx, ny) != (1.0, 2.0)
    assert nr != 0.2


def test_noisy_radius_never_goes_negative():
    rng = random.Random(1)
    for _ in range(500):
        _, _, nr = apply_noise(rng, 1.0, 1.0, 0.05, 3.0, MAX_RANGE, 0.03, 0.5)
        assert nr >= 0.0
