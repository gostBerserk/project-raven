"""Outputs of the drone program: flight controller (MAVLink), cold-gas board (serial), dry run.

Safety design: this program NEVER arms, takes off, lands or changes flight mode. The pilot
does those with the RC transmitter. Velocity commands are sent only while the vehicle is
armed and in GUIDED mode, and the pilot can take over at any time by switching mode."""
import time


class DryRunLink:
    """Records commands instead of sending them (default when no --connect is given)."""

    def __init__(self):
        self.sent = []

    def ready(self):
        return True

    def altitude(self):
        return None

    def send_velocity(self, vx_forward, vy_left):
        self.sent.append((vx_forward, vy_left))

    def close(self):
        pass


class MavlinkLink:
    """ArduPilot-style companion link: body-frame velocity set-points in GUIDED mode.
    Written against the pymavlink API; NOT yet run against a real flight controller or SITL."""

    BODY_VELOCITY_ONLY_MASK = 0b0000111111000111   # ignore position, acceleration, yaw, yaw rate

    def __init__(self, connection, baud=57600):
        from pymavlink import mavutil
        self._mu = mavutil
        self.m = mavutil.mavlink_connection(connection, baud=baud)
        if self.m.wait_heartbeat(timeout=10) is None:
            raise RuntimeError(f"No heartbeat from {connection}")
        self._alt = None

    def _poll(self):
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                return
            if msg.get_type() == "LOCAL_POSITION_NED":
                self._alt = -msg.z

    def ready(self):
        self._poll()
        return bool(self.m.motors_armed()) and self.m.flightmode == "GUIDED"

    def altitude(self):
        return self._alt

    def send_velocity(self, vx_forward, vy_left):
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            self._mu.mavlink.MAV_FRAME_BODY_OFFSET_NED, self.BODY_VELOCITY_ONLY_MASK,
            0, 0, 0,                      # position (ignored)
            vx_forward, -vy_left, 0.0,    # NED body frame: x forward, y RIGHT, z down (hold height)
            0, 0, 0,                      # acceleration (ignored)
            0, 0)                         # yaw, yaw rate (ignored)

    def close(self):
        self.m.close()


class RcsSerial:
    """Cold-gas board (firmware/drone_rcs). Protocol: FIRE:<body_heading_deg>:<pulse_ms>"""

    def __init__(self, port, baud=115200, cooldown_s=0.15):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)                    # the board resets when the port opens
        self.cooldown_s = cooldown_s
        self._last = 0.0

    def fire(self, heading_deg, pulse_ms=80):
        now = time.time()
        if now - self._last < self.cooldown_s:
            return False
        self._last = now
        self.ser.write(f"FIRE:{heading_deg:.1f}:{int(pulse_ms)}\n".encode("utf-8"))
        return True

    def close(self):
        self.ser.close()
