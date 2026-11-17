"""Safety-guarded driver for the Hiwonder xArm 1S.

Every servo command goes through SafeArm._send(), which:
  * refuses servos that have no configured limits,
  * clamps the target into the configured soft limits,
  * converts to a plain Python int (xarm treats floats as degrees and skips
    its range check for numpy ints),
  * stretches the move duration so no servo exceeds the hard speed cap.

The xArm controller only executes the LAST of several commands written
back-to-back (measured on the real arm), so multi-servo moves go out as one
packet and the USB backend spaces writes at least MIN_WRITE_GAP_S apart.

Smooth jogging is done by motion.MotionStreamer in its own thread, so every
USB access here is serialised with an I/O lock.

servoOff() is never called: it removes torque and the arm would drop.
"""

import json
import math
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config.json"

FULL_RANGE = 1000
MAX_DURATION_MS = 30000
MIN_WRITE_GAP_S = 0.03  # 20 ms measured as enough on the xArm 1S; keep a margin


class ArmSafetyError(RuntimeError):
    """Raised when a command would violate a safety rule."""


def load_config(path=None):
    with open(path or DEFAULT_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


class SimulatedBackend:
    """Stands in for the arm so the full pipeline can run with no hardware."""

    name = "Simulated arm"

    def __init__(self, start_positions=None):
        self.positions = dict(start_positions or {})
        self.commands = []
        self.packets = []

    def set_positions(self, positions, duration_ms):
        self.packets.append((dict(positions), duration_ms))
        for servo_id, position in positions.items():
            self.commands.append((servo_id, position, duration_ms))
            self.positions[servo_id] = position

    def get_position(self, servo_id):
        return self.positions.get(servo_id)

    def battery_voltage(self):
        return 7.5

    def close(self):
        pass


class XArmBackend:
    """Real xArm 1S over USB HID (needs the `xarm` and `hidapi` packages)."""

    name = "xArm 1S (USB)"

    def __init__(self, connection="USB"):
        import xarm  # imported here so simulation works without the library

        self._ctrl = xarm.Controller(connection)
        self._last_write = 0.0

    def _wait_gap(self):
        wait = MIN_WRITE_GAP_S - (time.monotonic() - self._last_write)
        if wait > 0:
            time.sleep(wait)

    def set_positions(self, positions, duration_ms):
        self._wait_gap()
        if len(positions) == 1:
            (servo_id, position), = positions.items()
            self._ctrl.setPosition(int(servo_id), int(position), duration=int(duration_ms), wait=False)
        else:
            pairs = [[int(sid), int(pos)] for sid, pos in positions.items()]
            self._ctrl.setPosition(pairs, duration=int(duration_ms), wait=False)
        self._last_write = time.monotonic()

    def get_position(self, servo_id, retries=3):
        # The HID read can time out and return an empty report; retry briefly.
        for _ in range(retries):
            self._wait_gap()
            try:
                return int(self._ctrl.getPosition(int(servo_id)))
            except Exception:
                time.sleep(0.05)
            finally:
                self._last_write = time.monotonic()
        return None

    def battery_voltage(self):
        self._wait_gap()
        try:
            return self._ctrl.getBatteryVoltage()
        except Exception:
            return None
        finally:
            self._last_write = time.monotonic()

    def close(self):
        try:
            self._ctrl._device.close()
        except Exception:
            pass


class SafeArm:
    """Position-tracking wrapper that enforces limits and speed caps."""

    def __init__(self, config, backend):
        arm = config["arm"]
        self.backend = backend
        self.limits = {int(k): (int(v["min"]), int(v["max"])) for k, v in arm["servos"].items()}
        self.names = {int(k): v.get("name", f"servo {k}") for k, v in arm["servos"].items()}
        self.home_pose = {int(k): int(v) for k, v in arm["home"].items()}
        self.home_min_ms = int(arm.get("home_min_duration_ms", 3000))
        self.hard_max_speed = float(arm["hard_max_speed_units_per_s"])
        self.command_ms = int(arm.get("command_duration_ms", 100))
        self.min_battery = float(arm.get("min_battery_voltage", 6.6))
        grip = arm["gripper"]
        self.grip_servo = int(grip["servo"])
        self.grip_open = int(grip["open"])
        self.grip_close = int(grip["close"])
        self.grip_ms = int(grip.get("move_ms", 700))

        self.speed = 0.0
        self._io_lock = threading.RLock()  # GUI thread and motion thread share the USB link
        self.targets = {}
        self._fine = {}
        self.homed = False
        self.gripper_closed = None
        self._validate()
        self.set_speed(float(arm["default_speed_units_per_s"]))

    def _validate(self):
        for sid, (lo, hi) in self.limits.items():
            if not 0 <= lo < hi <= FULL_RANGE:
                raise ArmSafetyError(f"Servo {sid}: invalid limits {lo}-{hi}")
        for sid, pos in self.home_pose.items():
            if sid not in self.limits:
                raise ArmSafetyError(f"Home uses servo {sid}, which has no limits")
            lo, hi = self.limits[sid]
            if not lo <= pos <= hi:
                raise ArmSafetyError(f"Home position {pos} for servo {sid} is outside {lo}-{hi}")
        if self.grip_servo not in self.limits:
            raise ArmSafetyError("Gripper servo has no limits")
        lo, hi = self.limits[self.grip_servo]
        for pos in (self.grip_open, self.grip_close):
            if not lo <= pos <= hi:
                raise ArmSafetyError(f"Gripper position {pos} is outside {lo}-{hi}")
        if self.hard_max_speed <= 0 or self.hard_max_speed > 600:
            raise ArmSafetyError("hard_max_speed_units_per_s must be between 0 and 600")

    # ------------------------------------------------------------------ core
    def clamp(self, servo_id, position):
        lo, hi = self.limits[servo_id]
        return int(round(min(hi, max(lo, position))))

    def set_speed(self, units_per_s):
        self.speed = max(5.0, min(float(units_per_s), self.hard_max_speed))
        return self.speed

    def _send(self, servo_id, position, duration_ms):
        return self._send_many({servo_id: position}, duration_ms)

    def _send_many(self, targets, duration_ms):
        """Guard every servo, then send them all in ONE packet with a shared
        duration long enough for the servo that travels furthest."""
        with self._io_lock:
            return self._send_many_locked(targets, duration_ms)

    def _send_many_locked(self, targets, duration_ms):
        packet = {}
        for servo_id, position in targets.items():
            servo_id = int(servo_id)
            if servo_id not in self.limits:
                raise ArmSafetyError(f"Servo {servo_id} is not allowed (no limits configured)")
            position = self.clamp(servo_id, position)
            previous = self.targets.get(servo_id)
            if previous == position:
                continue
            # Unknown start: assume it could be anywhere and move slowly.
            distance = abs(position - previous) if previous is not None else FULL_RANGE
            duration_ms = max(duration_ms, distance / self.hard_max_speed * 1000.0)
            packet[servo_id] = position
        if not packet:
            return 0
        duration_ms = int(min(MAX_DURATION_MS, math.ceil(duration_ms)))
        self.backend.set_positions(packet, duration_ms)
        for servo_id, position in packet.items():
            self.targets[servo_id] = position
            self._fine[servo_id] = float(position)
        return duration_ms

    # ------------------------------------------------------------- commands
    def read_positions(self):
        """Read every limited servo; seed targets so speed guards are accurate."""
        positions = {}
        with self._io_lock:
            for sid in self.limits:
                positions[sid] = self.backend.get_position(sid)
        for sid, pos in positions.items():
            if pos is not None:
                self.targets[sid] = int(pos)
                self._fine[sid] = float(pos)
        grip = positions.get(self.grip_servo)
        if grip is not None:
            self.gripper_closed = grip > (self.grip_open + self.grip_close) / 2
        return positions

    def go_home(self):
        """Move slowly to the home pose. Returns the move time in seconds."""
        duration = self._send_many(self.home_pose, self.home_min_ms)
        self.homed = True
        return max(duration, self.home_min_ms) / 1000.0

    def freeze(self):
        """Stop every servo where it physically is right now, even in the middle
        of a long move such as homing (halting the motion stream alone does not
        stop those). The gripper is left alone: while it holds an object it sits
        short of its target, and re-commanding that reading would loosen the grip.
        Returns the positions that were read."""
        with self._io_lock:
            positions = {sid: self.backend.get_position(sid) for sid in self.limits if sid != self.grip_servo}
            known = {sid: pos for sid, pos in positions.items() if pos is not None}
            if known:
                # The servos are already at these positions, so the speed-cap
                # stretch on the duration cannot cause any real motion.
                self._send_many_locked(known, 100)
        return positions

    def home_errors(self, tolerance=25):
        """Servos that are not at the home pose: {servo: (position or None, home)}."""
        with self._io_lock:
            readings = {sid: self.backend.get_position(sid) for sid in self.home_pose}
        return {sid: (pos, self.home_pose[sid]) for sid, pos in readings.items()
                if pos is None or abs(pos - self.home_pose[sid]) > tolerance}

    def setpoint(self, servo_id):
        """Float position the motion streamer is tracking for this servo."""
        servo_id = int(servo_id)
        if servo_id not in self.targets:
            raise ArmSafetyError(f"Servo {servo_id} position is unknown")
        return self._fine.get(servo_id, float(self.targets[servo_id]))

    def stream(self, setpoints, leads, duration_ms):
        """Send one streaming packet: each servo aims at setpoint + lead.
        Limits and the speed cap apply exactly as for any other command."""
        if not self.homed:
            raise ArmSafetyError("Press 'Go Home' before moving the arm")
        with self._io_lock:
            self._send_many_locked({sid: p + leads.get(sid, 0.0) for sid, p in setpoints.items()},
                                   duration_ms)
            for sid, p in setpoints.items():
                lo, hi = self.limits[int(sid)]
                self._fine[int(sid)] = min(hi, max(lo, float(p)))

    def hold(self, servo_ids):
        """Stop these servos where they are now (at their setpoint)."""
        with self._io_lock:
            targets = {int(s): self._fine.get(int(s), self.targets.get(int(s))) for s in servo_ids}
            targets = {s: p for s, p in targets.items() if p is not None}
            fine = {s: self._fine[s] for s in targets if s in self._fine}
            if targets:
                self._send_many_locked(targets, min(self.command_ms, 80))
            self._fine.update(fine)  # keep sub-unit precision for the next move

    def at_limit(self, servo_id, direction):
        lo, hi = self.limits[int(servo_id)]
        pos = self._fine.get(int(servo_id), self.targets.get(int(servo_id)))
        if pos is None:
            return False
        return pos >= hi if direction > 0 else pos <= lo

    def set_gripper(self, closed):
        if not self.homed:
            raise ArmSafetyError("Press 'Go Home' before using the gripper")
        self._send(self.grip_servo, self.grip_close if closed else self.grip_open, self.grip_ms)
        self.gripper_closed = bool(closed)
        return self.gripper_closed

    def toggle_gripper(self):
        return self.set_gripper(not self.gripper_closed)

    def battery(self):
        """Returns (ok, voltage). Unknown voltage counts as ok."""
        with self._io_lock:
            voltage = self.backend.battery_voltage()
        if voltage is None or voltage <= 0.5:
            return True, None
        return voltage >= self.min_battery, voltage

    def close(self):
        with self._io_lock:
            self.backend.close()
