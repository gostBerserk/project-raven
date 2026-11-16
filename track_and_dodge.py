"""
Thruster Platform - Track & Dodge Controller
=============================================

Overhead camera (webcam) looks straight down at the platform on its
smooth surface. This script:
  1. Tracks the PLATFORM itself (a colored marker on top of it).
  2. Tracks a colored TARGET object and computes the heading the platform
     should move to reach and touch it.
  3. Tracks colored INCOMING OBJECTS (foam balls), estimates their velocity
     relative to the platform, and checks for a collision course.
  4. Decides a direction of travel: dodge takes priority over chasing.
     A dodge moves SIDEWAYS (perpendicular to the ball's path), which gets
     out of the way faster than backing off along the ball's line.
  5. Sends "FIRE:<heading_deg>:<pulse_ms>" over serial. The firmware
     (firmware/thruster_controller) turns that into valve pulses, using the
     IMU heading so the command is in TABLE coordinates, not platform ones.

Heading convention: degrees, 0 = right (east) on the camera image,
90 = up on the image, counter-clockwise positive.

Dependencies: opencv-python, numpy, pyserial
    pip install opencv-python numpy pyserial
"""

import math
import os
import sys
import time

import cv2
import numpy as np
import serial

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gnc import params                      # noqa: E402  shared with the simulation
from gnc.estimator import TrackedPoint      # noqa: E402
from gnc.guidance import dodge_vector       # noqa: E402

# ----------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------

SERIAL_PORT = "/dev/ttyUSB0"   # change to your Arduino/ESP32 port (e.g. "COM5" on Windows)
BAUD_RATE = 115200
CAMERA_INDEX = 0

# HSV color ranges - placeholders, tune these under your own lighting.
# OpenCV hue runs 0..179.
PLATFORM_HSV_LOW = np.array([100, 120, 80])     # e.g. a blue marker on top of the platform
PLATFORM_HSV_HIGH = np.array([130, 255, 255])

TARGET_HSV_LOW = np.array([40, 80, 80])         # e.g. green
TARGET_HSV_HIGH = np.array([80, 255, 255])

# Red wraps around the hue circle (both ends of 0..179), so it needs TWO ranges.
# Add an orange range here too (about 10..25) if your balls look orange.
THREAT_HSV_RANGES = [
    (np.array([0, 120, 120]), np.array([10, 255, 255])),
    (np.array([170, 120, 120]), np.array([179, 255, 255])),
]

# True: follow the platform marker (needed for a platform that moves).
# False: assume the platform sits at the image center (desk testing only).
USE_PLATFORM_MARKER = True

MIN_BLOB_AREA = 150            # px^2, filters noise
PLATFORM_RADIUS_PX = 60        # platform's visible radius on screen, for collision margin
COLLISION_TIME_HORIZON = params.HORIZON_S   # seconds, shared with the simulation (gnc/params.py)
TOUCH_DISTANCE_PX = 40         # how close to target counts as "touched"
ASSOCIATION_GATE_PX = 80       # max jump when matching a ball to its previous position

# ----------------------------------------------------------------------
# TRACKING HELPERS
# ----------------------------------------------------------------------

def clean_mask(mask):
    mask = cv2.erode(mask, None, iterations=2)
    return cv2.dilate(mask, None, iterations=2)


def find_largest_blob(mask):
    """Returns (cx, cy, area) of the largest contour in a binary mask, or None."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_BLOB_AREA:
        return None
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    return M["m10"] / M["m00"], M["m01"] / M["m00"], cv2.contourArea(c)


def find_all_blobs(mask, min_area=MIN_BLOB_AREA):
    """Returns list of (cx, cy, area) for every contour above min_area."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        blobs.append((M["m10"] / M["m00"], M["m01"] / M["m00"], area))
    return blobs


def threat_mask_from_hsv(hsv):
    mask = None
    for low, high in THREAT_HSV_RANGES:
        m = cv2.inRange(hsv, low, high)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return clean_mask(mask)


# ----------------------------------------------------------------------
# COLLISION PREDICTION
# ----------------------------------------------------------------------

def best_dodge_direction(px, py, pvx, pvy, threats, target=None):
    """Image-pixel wrapper around gnc.guidance.dodge_vector (the same code the
    simulation runs). Image y points down, so y is flipped on the way in.
    Returns a heading in degrees (0 = east, CCW positive) or None if no dodge is needed."""
    res = dodge_vector(
        (px, -py), (pvx, -pvy),
        [(ox, -oy, ovx, -ovy) for (ox, oy, ovx, ovy) in threats],
        PLATFORM_RADIUS_PX,
        target=None if target is None else (target[0], -target[1]),
        horizon=COLLISION_TIME_HORIZON,
    )
    if res is None:
        return None
    return math.degrees(math.atan2(res[1], res[0])) % 360


def heading_to_target(px, py, tx, ty):
    return math.degrees(math.atan2(-(ty - py), tx - px)) % 360   # flip y for screen coords


# ----------------------------------------------------------------------
# SERIAL COMMAND
# ----------------------------------------------------------------------

def send_thruster_command(ser, heading_deg, pulse_ms=60):
    """
    Protocol: "FIRE:<heading_deg>:<pulse_ms>\\n"
    heading_deg is the desired direction of travel in table coordinates.
    The firmware converts it to the platform's frame using the IMU and opens
    the best-matching valve(s) for pulse_ms (it clamps long pulses and refuses
    to fire until the IMU is calibrated).
    Keep pulses short (40-80 ms): thrust is on/off (bang-bang), not proportional.
    """
    ser.write(f"FIRE:{heading_deg:.1f}:{int(pulse_ms)}\n".encode("utf-8"))


# ----------------------------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------------------------

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)  # let the board reset after serial connect
    except Exception as e:
        print(f"[WARN] No serial connection ({e}) - running in simulate-only mode.")
        ser = None

    platform_track = TrackedPoint()
    threat_tracks = {}  # id -> TrackedPoint
    next_threat_id = 0
    last_fire_time = 0.0
    FIRE_COOLDOWN = 0.15  # seconds, avoid spamming the valves

    print("Press 'q' to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        now = time.time()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]

        # --- find the platform ---
        platform_ok = True
        if USE_PLATFORM_MARKER:
            blob = find_largest_blob(clean_mask(cv2.inRange(hsv, PLATFORM_HSV_LOW, PLATFORM_HSV_HIGH)))
            if blob is not None:
                platform_track.update(blob[0], blob[1], now)
            last_seen = platform_track.last_time()
            if last_seen is None or now - last_seen > 0.3:
                platform_ok = False        # marker lost: do NOT fire blind
            if platform_ok:
                px, py = platform_track.position()
                pvx, pvy = platform_track.velocity()
        else:
            px, py, pvx, pvy = w / 2, h / 2, 0.0, 0.0

        # --- track target ---
        target_blob = find_largest_blob(clean_mask(cv2.inRange(hsv, TARGET_HSV_LOW, TARGET_HSV_HIGH)))

        # --- track threats (foam balls) ---
        threat_blobs = find_all_blobs(threat_mask_from_hsv(hsv))
        assigned = set()
        for (bx, by, _area) in threat_blobs:
            best_id, best_dist = None, 1e9
            for tid, track in threat_tracks.items():
                if tid in assigned:
                    continue                   # one blob per track
                pos = track.position()
                d = math.hypot(pos[0] - bx, pos[1] - by)
                if d < best_dist and d < ASSOCIATION_GATE_PX:
                    best_dist, best_id = d, tid
            if best_id is None:
                best_id = next_threat_id
                next_threat_id += 1
                threat_tracks[best_id] = TrackedPoint()
            threat_tracks[best_id].update(bx, by, now)
            assigned.add(best_id)

        for tid in list(threat_tracks.keys()):     # drop stale tracks
            if now - threat_tracks[tid].last_time() > 0.5:
                del threat_tracks[tid]

        threats = []
        for tid in assigned:
            track = threat_tracks[tid]
            pos = track.position()
            vx, vy = track.velocity()
            threats.append((pos[0], pos[1], vx, vy))

        # --- decide action: dodge beats chase ---
        action = None
        if not platform_ok:
            action = ("PLATFORM LOST", None)
        else:
            tpos = (target_blob[0], target_blob[1]) if target_blob else None
            dodge_heading = best_dodge_direction(px, py, pvx, pvy, threats, target=tpos)
            if dodge_heading is not None:
                action = ("DODGE", dodge_heading)
            elif tpos is not None:
                if math.hypot(tpos[0] - px, tpos[1] - py) > TOUCH_DISTANCE_PX:
                    action = ("CHASE", heading_to_target(px, py, *tpos))
                else:
                    action = ("TOUCHED", None)

        # --- fire thrusters ---
        if action and action[0] in ("DODGE", "CHASE") and (now - last_fire_time) > FIRE_COOLDOWN:
            heading = action[1]
            pulse = 80 if action[0] == "DODGE" else 50  # harder pulse to evade
            if ser:
                send_thruster_command(ser, heading, pulse_ms=pulse)
            last_fire_time = now
            print(f"{action[0]} heading={heading:.0f} deg pulse={pulse} ms")
        elif action and action[0] == "TOUCHED":
            print("Target touched.")

        # --- visualization ---
        if target_blob:
            cv2.circle(frame, (int(target_blob[0]), int(target_blob[1])), 10, (0, 255, 0), 2)
        for (bx, by, vx, vy) in threats:
            cv2.circle(frame, (int(bx), int(by)), 10, (0, 0, 255), 2)
            cv2.arrowedLine(frame, (int(bx), int(by)),
                            (int(bx + vx * 0.2), int(by + vy * 0.2)), (0, 0, 255), 2)
        if platform_ok:
            cv2.circle(frame, (int(px), int(py)), PLATFORM_RADIUS_PX, (255, 255, 0), 1)
        if action:
            cv2.putText(frame, str(action[0]), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv2.imshow("Track & Dodge", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()


if __name__ == "__main__":
    main()
