"""Run from the project root:  python -m unittest discover tests -v
Covers the shared guidance and estimator code (no camera, serial or plotting needed)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np                                      # noqa: E402
from gnc.estimator import TrackedPoint                  # noqa: E402
from gnc.guidance import dodge_vector, time_to_closest_approach   # noqa: E402

R = 60.0                 # platform radius, same units as positions (pixels here)
P = (320.0, 240.0)       # platform position, y UP


def heading(res):
    return None if res is None else math.degrees(math.atan2(res[1], res[0])) % 360


def dodge(threats, pv=(0.0, 0.0), target=None):
    return heading(dodge_vector(P, pv, threats, R, target=target))


def angle_close(a, b, tol=1.0):
    return abs((a - b + 180) % 360 - 180) < tol


class GuidanceTests(unittest.TestCase):
    def test_closest_approach_head_on(self):
        t, d = time_to_closest_approach(300, 0, -300, 0)
        self.assertAlmostEqual(t, 1.0)
        self.assertAlmostEqual(d, 0.0)

    def test_no_relative_motion(self):
        self.assertIsNone(time_to_closest_approach(10, 0, 0, 0)[0])

    def test_ball_passing_below_dodges_up(self):      # y up: ball at y-20 passes below
        self.assertTrue(angle_close(dodge([(620, 220, -300, 0)]), 90))

    def test_ball_passing_above_dodges_down(self):
        self.assertTrue(angle_close(dodge([(620, 260, -300, 0)]), 270))

    def test_ball_from_above_passing_left_dodges_right(self):
        self.assertTrue(angle_close(dodge([(300, 300, 0, -300)]), 0))

    def test_dead_on_steps_toward_target_side(self):
        self.assertTrue(angle_close(dodge([(620, 240, -300, 0)], target=(320, 80)), 270))
        self.assertTrue(angle_close(dodge([(620, 240, -300, 0)], target=(320, 400)), 90))

    def test_far_miss_ignored(self):
        self.assertIsNone(dodge([(620, 40, -300, 0)]))

    def test_too_far_in_time_ignored(self):
        self.assertIsNone(dodge([(900, 230, -300, 0)]))

    def test_moving_with_the_ball_is_safe(self):
        self.assertIsNone(dodge([(620, 220, -300, 0)], pv=(-300.0, 0.0)))

    def test_opposing_threats_still_give_a_heading(self):
        self.assertIsNotNone(dodge([(620, 220, -300, 0), (20, 260, 300, 0)]))

    def test_urgency_between_0_and_1(self):
        res = dodge_vector(P, (0, 0), [(620, 220, -300, 0)], R)
        self.assertTrue(0.0 <= res[2] <= 1.0)


class EstimatorTests(unittest.TestCase):
    def test_empty_and_single_point(self):
        tp = TrackedPoint()
        self.assertEqual(tp.velocity(), (0.0, 0.0))
        tp.update(1, 1, 0.0)
        self.assertEqual(tp.velocity(), (0.0, 0.0))

    def test_exact_velocity_without_noise(self):
        tp = TrackedPoint()
        for k in range(8):
            tp.update(5 + 2.0 * k / 30, 1 - 0.5 * k / 30, k / 30)
        vx, vy = tp.velocity()
        self.assertAlmostEqual(vx, 2.0, places=6)
        self.assertAlmostEqual(vy, -0.5, places=6)

    def test_noisy_fit_beats_endpoint_difference(self):
        rng = np.random.default_rng(1)
        fit_err, end_err = [], []
        for _ in range(200):
            tp = TrackedPoint(maxlen=10)
            for k in range(10):
                tp.update(0.4 * k / 30 + rng.normal(0, 0.008), 0.0, k / 30)
            fit_err.append(abs(tp.velocity()[0] - 0.4))
            (x0, _, t0), (x1, _, t1) = tp.history[0], tp.history[-1]
            end_err.append(abs((x1 - x0) / (t1 - t0) - 0.4))
        self.assertLess(np.mean(fit_err), np.mean(end_err))


class SimulationSmokeTest(unittest.TestCase):
    def test_short_run_completes(self):
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "simulation"))
        import thruster_sim as ts
        ts.T_END, ts.rng = 4.0, np.random.default_rng(0)
        log, dist_t, min_ball, touches, hits = ts.run()
        self.assertEqual(len(dist_t), int(4.0 / ts.DT))
        self.assertTrue(np.all(np.isfinite(dist_t)))


if __name__ == "__main__":
    unittest.main()
