"""Pure conversion regressions; does not start ROS, drivers or a robot."""
import math
import unittest

from luna_control.drive_controller import wheel_targets_qpps


class DriveControllerTest(unittest.TestCase):
    # Test-only geometry, not a claimed physical rover calibration.
    geometry = (0.6, 0.15, 4096.0, 0.1)

    def convert(self, v, w):
        return wheel_targets_qpps(v, w, *self.geometry)

    def test_stationary_stays_zero(self):
        self.assertEqual(self.convert(0, 0), (0, 0))

    def test_forward_backward_are_symmetric(self):
        a, b = self.convert(0.05, 0)
        self.assertGreater(a, 0)
        self.assertEqual(a, b)
        self.assertEqual(self.convert(-0.05, 0), (-a, -b))

    def test_pure_turn_produces_opposite_signs(self):
        a, b = self.convert(0, 0.1)
        self.assertLess(a, 0)
        self.assertEqual(b, -a)

    def test_curve_saturation_preserves_ratio(self):
        a, b = self.convert(1, 0.5)
        self.assertGreater(a, 0)
        self.assertGreater(b, a)
        max_qpps = round(self.geometry[3] *
                         self.geometry[2] / (2 * math.pi * self.geometry[1]))
        self.assertLessEqual(max(abs(a), abs(b)), max_qpps)

    def test_small_commands_do_not_jump_to_minimum_speed(self):
        a, b = self.convert(0.0001, 0)
        self.assertTrue(0 <= a <= 1)
        self.assertEqual(a, b)

    def test_rejects_uncalibrated_or_invalid(self):
        for bad in (0, -1, float("nan"), float("inf")):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                wheel_targets_qpps(0, 0, .6, .15, 4096, bad)
        with self.assertRaises(ValueError):
            self.convert(float("nan"), 0)


if __name__ == "__main__":
    unittest.main()
