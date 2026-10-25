
import math

from luna_sim.sensors.wheel_model import WheelOdometryModel

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
