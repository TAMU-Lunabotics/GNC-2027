
import math

from luna_sim.sensors.wheel_model import WheelOdometryModel
from luna_sim.sensors.wheel_model import expand_covariance

# These basic unit tests were LLM generated


def make_model():
    return WheelOdometryModel(
        wheel_radius=0.15,
        wheel_separation=0.6,
        ticks_per_rev=4096,
        noise_std=0.0,
        noise_per_rad_s=0.0,
        seed=42,
    )


def test_straight_line():
    model = make_model()

    for _ in range(50):
        result = model.step(true_v=0.5, true_w=0.0, dt=0.02)

    assert math.isclose(result.x, 0.5, abs_tol=0.001)
    assert math.isclose(result.y, 0.0, abs_tol=0.001)
    assert math.isclose(result.yaw, 0.0, abs_tol=0.001)
    assert result.left_ticks == result.right_ticks


def test_rotation_in_place():
    model = make_model()

    for _ in range(50):
        result = model.step(true_v=0.0, true_w=1.0, dt=0.02)

    assert math.isclose(result.yaw, 1.0, abs_tol=0.001)
    assert math.isclose(result.x, 0.0, abs_tol=0.001)
    assert math.isclose(result.y, 0.0, abs_tol=0.001)
    assert result.left_ticks == -result.right_ticks


def test_reproducibility():
    a = WheelOdometryModel(0.15, 0.6, noise_std=0.02, seed=42)
    b = WheelOdometryModel(0.15, 0.6, noise_std=0.02, seed=42)

    for _ in range(100):
        result_a = a.step(0.4, 0.5, 0.02)
        result_b = b.step(0.4, 0.5, 0.02)

        assert result_a == result_b


# ROS covariance embedding should preserve planar terms and row-major layout.

def test_pose_covariance_embedding():
    cov = expand_covariance([[1.0, 0.2, 0.3],
                             [0.2, 2.0, 0.4],
                             [0.3, 0.4, 3.0]], (0, 1, 5))
    assert len(cov) == 36
    assert (cov[0], cov[7], cov[35]) == (1.0, 2.0, 3.0)
    assert cov[5] == cov[30] == 0.3
    assert cov[2 * 6 + 2] == 1e6


def test_twist_covariance_embedding():
    cov = expand_covariance([[0.1, -0.02], [-0.02, 0.2]], (0, 5))
    assert cov[0] == 0.1
    assert cov[35] == 0.2
    assert cov[5] == cov[30] == -0.02
