"""Drone pilot program: camera -> tracking -> follow-and-dodge -> flight controller.

A downward-facing camera (image-up = drone forward) watches a colored TARGET (e.g. a person
carrying a green marker) and colored incoming BALLS. The program:
  1. finds target and balls (same HSV code as the table platform, vision/track_and_dodge.py),
  2. converts pixels to body-frame metres using the drone's height and the lens field of view,
  3. keeps the target at a standoff distance and sidesteps balls (drone/planner.py, which
     uses the same gnc/guidance.py as the simulation),
  4. sends a body-frame velocity set-point to the flight controller, and optionally pulses
     the 6-nozzle cold-gas board on urgent dodges.

SAFETY: it does not arm, take off, land or change mode. Fly with the RC transmitter in
hand and switch out of GUIDED to take over. Default is a DRY RUN (nothing is sent).
See docs/DRONE.md before flying anything.

Usage
    python drone/pilot.py --altitude 4 --show                       # dry run, webcam
    python drone/pilot.py --connect udp:127.0.0.1:14550 --altitude 4   # ArduPilot SITL
"""
import argparse
import math
import os
import sys
import time

import cv2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from gnc.association import ThreatTracker                       # noqa: E402
from vision import track_and_dodge as tv                        # noqa: E402  HSV ranges + blob helpers
from drone.planner import PilotConfig, PixelToBody, plan        # noqa: E402
from drone.links import DryRunLink, MavlinkLink, RcsSerial      # noqa: E402

TARGET_LOST_S = 0.5       # forget the target after this long unseen
SEND_PERIOD_S = 0.05      # 20 Hz set-points
FRAME_TIMEOUT_S = 0.3     # no new frame for this long -> command zero velocity
RCS_MIN_URGENCY = 0.5     # only pulse the cold-gas board for dodges at least this urgent


class Pilot:
    def __init__(self, cfg, width, height, hfov_deg, altitude_m):
        self.cfg = cfg
        self.convert = PixelToBody(width, height, hfov_deg, altitude_m)
        self.balls = ThreatTracker(gate=0.4)            # metres between frames
        self.target = None
        self.target_seen = -1e9

    def set_altitude(self, altitude_m, width, height, hfov_deg):
        self.convert = PixelToBody(width, height, hfov_deg, altitude_m)

    def step(self, frame_bgr, now):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        tb = tv.find_largest_blob(tv.clean_mask(cv2.inRange(hsv, tv.TARGET_HSV_LOW, tv.TARGET_HSV_HIGH)))
        if tb is not None:
            self.target = self.convert(tb[0], tb[1])
            self.target_seen = now
        elif now - self.target_seen > TARGET_LOST_S:
            self.target = None

        blobs = tv.find_all_blobs(tv.threat_mask_from_hsv(hsv))
        threats = self.balls.update([self.convert(b[0], b[1]) for b in blobs], now)
        return plan(self.target, threats, self.cfg), threats


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", help="MAVLink connection, e.g. udp:127.0.0.1:14550 or /dev/ttyACM0. Omit for a dry run.")
    ap.add_argument("--rcs-port", help="serial port of the 6-nozzle cold-gas board (optional)")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--altitude", type=float, default=4.0, help="metres, used if the flight controller does not report it")
    ap.add_argument("--hfov", type=float, default=80.0, help="camera horizontal field of view, degrees")
    ap.add_argument("--standoff", type=float, default=2.0, help="metres to keep from the target")
    ap.add_argument("--max-speed", type=float, default=1.5, help="m/s cap on any command")
    ap.add_argument("--show", action="store_true", help="show the camera view (needs a display)")
    args = ap.parse_args()

    cfg = PilotConfig(max_speed=args.max_speed, standoff_m=args.standoff)
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera")
    link = MavlinkLink(args.connect) if args.connect else DryRunLink()
    rcs = RcsSerial(args.rcs_port) if args.rcs_port else None
    if not args.connect:
        print("[DRY RUN] nothing is sent to a flight controller.")

    pilot = None
    last_send, last_frame, last_msg = 0.0, time.time(), 0.0
    try:
        while True:
            ok, frame = cap.read()
            now = time.time()
            if not ok:
                if now - last_frame > FRAME_TIMEOUT_S:
                    link.send_velocity(0.0, 0.0)        # camera dropped out: stop moving
                if now - last_frame > 2.0:
                    break
                continue
            last_frame = now
            h, w = frame.shape[:2]
            alt = link.altitude() or args.altitude
            if pilot is None:
                pilot = Pilot(cfg, w, h, args.hfov, alt)
            else:
                pilot.set_altitude(alt, w, h, args.hfov)
            p, threats = pilot.step(frame, now)

            if link.ready():
                if now - last_send >= SEND_PERIOD_S:
                    link.send_velocity(p.vx, p.vy)
                    last_send = now
                if rcs and p.mode == "DODGE" and p.urgency >= RCS_MIN_URGENCY:
                    rcs.fire(p.heading_deg)
            elif now - last_msg > 2.0:
                print("Waiting: vehicle must be armed and in GUIDED mode (the pilot does that).")
                last_msg = now

            if args.connect is None and p.mode != "HOLD":
                print(f"{p.mode:6s} vx={p.vx:+.2f} vy={p.vy:+.2f} m/s  urgency={p.urgency:.2f}")
            if args.show:
                cv2.putText(frame, p.mode, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow("drone pilot", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        link.send_velocity(0.0, 0.0)                    # always leave with a stop command
        link.close()
        if rcs:
            rcs.close()
        cap.release()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
