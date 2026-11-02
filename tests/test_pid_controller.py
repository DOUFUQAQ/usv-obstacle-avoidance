"""PID 控制器与轨迹跟踪器。"""
import math
import unittest

from src.control.pid_controller import PIDController, TrajectoryTracker


class TestPIDController(unittest.TestCase):
    def test_proportional_term_dominates_first_step(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        self.assertAlmostEqual(pid.update(error=1.0, current_time=0.0), 2.0)

    def test_output_sign_follows_error_sign(self):
        pid = PIDController(kp=1.0, ki=0.01, kd=0.0)
        self.assertGreater(pid.update(error=3.0, current_time=0.0), 0.0)

        pid.reset()
        self.assertLess(pid.update(error=-3.0, current_time=0.0), 0.0)

    def test_integral_accumulates_over_time(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        first = pid.update(error=1.0, current_time=0.0)     # 首次调用使用默认 dt=0.1
        second = pid.update(error=1.0, current_time=0.1)    # 积分项再累加一次

        self.assertAlmostEqual(first, 0.1, places=6)
        self.assertAlmostEqual(second, 0.2, places=6)

    def test_output_is_clamped(self):
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0, output_limit=(-1.0, 1.0))
        self.assertAlmostEqual(pid.update(error=5.0, current_time=0.0), 1.0)

        pid.reset()
        self.assertAlmostEqual(pid.update(error=-5.0, current_time=0.0), -1.0)

    def test_reset_clears_internal_state(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=0.0)
        pid.update(error=1.0, current_time=0.0)
        pid.reset()

        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.previous_error, 0.0)
        self.assertIsNone(pid.previous_time)

    def test_zero_error_gives_zero_output(self):
        pid = PIDController(kp=3.0, ki=0.5, kd=0.2)
        self.assertAlmostEqual(pid.update(error=0.0, current_time=0.0), 0.0)


class TestTrajectoryTracker(unittest.TestCase):
    def setUp(self):
        self.tracker = TrajectoryTracker()

    def test_goal_reached_uses_tolerance(self):
        tolerance = self.tracker.goal_tolerance
        self.assertTrue(
            self.tracker.is_goal_reached((0.0, 0.0, 0.0), (tolerance * 0.5, 0.0)))
        self.assertFalse(
            self.tracker.is_goal_reached((0.0, 0.0, 0.0), (tolerance * 2.0, 0.0)))

    def test_angle_normalisation_wraps_into_pi_range(self):
        self.assertAlmostEqual(self.tracker._normalize_angle(3 * math.pi), math.pi, places=9)
        self.assertAlmostEqual(self.tracker._normalize_angle(-3 * math.pi), -math.pi, places=9)
        self.assertAlmostEqual(self.tracker._normalize_angle(0.5), 0.5)


if __name__ == "__main__":
    unittest.main()
