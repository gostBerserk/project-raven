"""Drone integration tests. Run from the project root:  python -m unittest discover tests -v"""
import math
import os
import re
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import numpy as np                                            # noqa: E402
from gnc.association import ThreatTracker                     # noqa: E402
from drone import rcs                                         # noqa: E402
from drone.planner import PilotConfig, PixelToBody, plan, follow_velocity   # noqa: E402

CFG = PilotConfig()


class RcsTests(unittest.TestCase):
    def test_python_table_matches_firmware(self):
        with open(os.path.join(ROOT, "firmware", "drone_rcs", "drone_rcs.ino")) as f:
            src = f.read()
        m = re.search(r"VALVE_PUSH_DEG\[NUM_VALVES\]\s*=\s*\{([^}]*)\}", src)
        fw = [float(x) for x in m.group(1).split(",")]
        self.assertEqual(fw, rcs.PUSH_DEG)
        self.assertEqual(int(re.search(r"NUM_VALVES\s*=\s*(\d+)", src).group(1)), len(rcs.PUSH_DEG))
        self.assertEqual(float(re.search(r"MAX_ANGLE_DIFF\s*=\s*([\d.]+)", src).group(1)), rcs.MAX_ANGLE_DIFF)

    def test_axis_headings_pick_expected_valves(self):
        self.assertEqual(rcs.select_valves(0), [5])           # forward -> back-centre
        self.assertEqual(rcs.select_valves(180), [4])         # back -> front-centre
        self.assertEqual(rcs.select_valves(90), [1, 3])       # left -> the two valves that exhaust right
        self.assertEqual(rcs.select_valves(270), [0, 2])      # right -> the two valves that exhaust left

    def test_net_push_roughly_follows_heading(self):
        for h in range(0, 360, 15):
            net = rcs.net_push_deg(rcs.select_valves(h))
            self.assertLessEqual(rcs.ang_diff(net, h), 35.0, f"heading {h} -> net {net:.0f}")


class SpinTests(unittest.TestCase):
    def test_no_heading_makes_the_drone_spin(self):
        for h in range(0, 360, 5):
            self.assertAlmostEqual(rcs.yaw_torque(rcs.select_valves(h)), 0.0, places=9, msg=f"heading {h}")

    def test_wing_mounted_fore_aft_nozzles_would_spin_it(self):
        wing = list(rcs.POSITION_M)
        wing[4], wing[5] = (0.0, 0.30), (0.0, -0.30)          # the earlier, wrong placement
        self.assertGreater(abs(rcs.yaw_torque([4], wing)), 0.1)
        self.assertGreater(abs(rcs.yaw_torque([5], wing)), 0.1)
        self.assertGreater(abs(rcs.yaw_torque([4, 5], wing)), 0.1)   # together: a pure couple


class PlannerTests(unittest.TestCase):
    def test_far_target_follows_forward(self):
        p = plan((6.0, 0.0), [], CFG)
        self.assertEqual(p.mode, "FOLLOW")
        self.assertGreater(p.vx, 0)
        self.assertAlmostEqual(p.vy, 0.0)

    def test_speed_is_capped(self):
        p = plan((50.0, 30.0), [], CFG)
        self.assertLessEqual(math.hypot(p.vx, p.vy), CFG.max_speed + 1e-9)

    def test_holds_at_standoff(self):
        self.assertEqual(plan((CFG.standoff_m, 0.0), [], CFG).mode, "HOLD")

    def test_backs_off_when_too_close(self):
        vx, vy = follow_velocity((1.0, 0.0), CFG)
        self.assertLess(vx, 0)                                 # moves away from the target

    def test_never_approaches_inside_standoff(self):
        for dist in np.linspace(0.3, CFG.standoff_m, 12):
            vx, vy = follow_velocity((dist, 0.0), CFG)
            self.assertLessEqual(vx, 0.0)

    def test_no_target_no_threat_holds(self):
        self.assertEqual(plan(None, [], CFG).mode, "HOLD")

    def test_ball_passing_on_left_dodges_right(self):
        # ball ahead and slightly left (y > 0), flying straight at the drone's position
        p = plan(None, [(3.0, 0.15, -2.5, 0.0)], CFG)
        self.assertEqual(p.mode, "DODGE")
        self.assertLess(p.vy, 0)                               # right
        self.assertLess(abs(p.vx), abs(p.vy))                  # sideways, not along the ball's path

    def test_dodge_beats_follow(self):
        p = plan((6.0, 0.0), [(3.0, 0.15, -2.5, 0.0)], CFG)
        self.assertEqual(p.mode, "DODGE")

    def test_distant_ball_is_ignored(self):
        self.assertEqual(plan((6.0, 0.0), [(3.0, 4.0, -2.5, 0.0)], CFG).mode, "FOLLOW")

    def test_dodge_heading_matches_velocity(self):
        p = plan(None, [(3.0, 0.15, -2.5, 0.0)], CFG)
        self.assertAlmostEqual(rcs.ang_diff(p.heading_deg, math.degrees(math.atan2(p.vy, p.vx)) % 360), 0.0, places=6)


class PixelToBodyTests(unittest.TestCase):
    def test_centre_is_origin(self):
        self.assertEqual(PixelToBody(640, 480, 80, 4.0)(320, 240), (0.0, 0.0))

    def test_axes_and_scale(self):
        c = PixelToBody(640, 480, 90, 4.0)                    # footprint is 8 m wide -> 0.0125 m/px
        fwd, left = c(0, 240)                                 # left edge of the image
        self.assertAlmostEqual(left, 4.0)
        self.assertAlmostEqual(fwd, 0.0)
        fwd, left = c(320, 0)                                 # top edge of the image
        self.assertAlmostEqual(fwd, 3.0)                      # 240 px * 0.0125
        self.assertAlmostEqual(left, 0.0)


class TrackerTests(unittest.TestCase):
    def test_follows_two_objects_and_estimates_velocity(self):
        tr = ThreatTracker(gate=0.4)
        out = None
        for k in range(12):
            t = k / 30
            out = tr.update([(2.0 - 1.5 * t, 0.5), (-1.0, -2.0 + 0.6 * t)], t)
        self.assertEqual(len(out), 2)
        speeds = sorted(round(math.hypot(o[2], o[3]), 1) for o in out)
        self.assertEqual(speeds, [0.6, 1.5])

    def test_stale_tracks_are_dropped(self):
        tr = ThreatTracker(gate=0.4, stale_s=0.5)
        tr.update([(0.0, 0.0)], 0.0)
        tr.update([], 1.0)
        self.assertEqual(len(tr.tracks), 0)


class PilotFrameTest(unittest.TestCase):
    def test_target_in_synthetic_frame(self):
        try:
            import cv2
            from drone.pilot import Pilot
        except Exception as e:                                # opencv/serial missing
            self.skipTest(str(e))
        h, s, v = 60, 200, 200
        bgr = tuple(int(c) for c in cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0])
        frame = np.zeros((480, 640, 3), np.uint8)
        cv2.circle(frame, (480, 100), 20, bgr, -1)            # green target, upper right
        pilot = Pilot(CFG, 640, 480, 80.0, 4.0)
        p, threats = pilot.step(frame, 1.0)
        self.assertEqual(p.mode, "FOLLOW")
        self.assertGreater(p.vx, 0)                           # target is ahead
        self.assertLess(p.vy, 0)                              # and to the right
        self.assertEqual(threats, [])


class LinkTests(unittest.TestCase):
    def test_dry_run_records(self):
        from drone.links import DryRunLink
        link = DryRunLink()
        link.send_velocity(1.0, -0.5)
        self.assertEqual(link.sent, [(1.0, -0.5)])

    def test_mavlink_message_layout(self):
        try:
            from pymavlink import mavutil
        except Exception as e:
            self.skipTest(str(e))
        from drone.links import MavlinkLink
        msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, 1, 1, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, MavlinkLink.BODY_VELOCITY_ONLY_MASK,
            0, 0, 0, 1.0, 0.5, 0.0, 0, 0, 0, 0, 0)
        ignored = lambda bit: bool(msg.type_mask >> bit & 1)
        self.assertTrue(all(ignored(b) for b in (0, 1, 2)))        # position ignored
        self.assertFalse(any(ignored(b) for b in (3, 4, 5)))       # velocity used
        self.assertTrue(all(ignored(b) for b in (6, 7, 8, 10, 11)))  # accel, yaw, yaw rate ignored


class SimSmokeTest(unittest.TestCase):
    def test_short_run(self):
        from drone import sim
        sim.T_END = 3.0
        hits, err = sim.run(0)
        self.assertGreaterEqual(hits, 0)
        self.assertTrue(math.isfinite(err))


if __name__ == "__main__":
    unittest.main()
