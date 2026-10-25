"""Smooth, continuous joint motion for the xArm 1S.

The xArm servos take "go to P within T ms" commands, and each new command
restarts the move from wherever the servo is. The old approach sent a tiny step
from the GUI timer on every tick. The step size and timing then depended on GUI
load, and the blocking USB writes ran on the GUI thread, so motion was jerky and
laggy.

MotionStreamer runs in its own thread at a fixed rate. Callers only set a
desired velocity per servo (units/s). Every cycle it:
  * ramps each servo's velocity toward the desired one (acceleration limit),
  * moves a float setpoint by velocity * dt, clamped to the soft limits,
  * sends ONE packet that aims every moving servo `lookahead` ahead of its
    setpoint, with duration = lookahead. The servo then moves at the commanded
    speed and never comes to rest between packets.
A servo whose velocity reaches zero gets one last command to its setpoint.
If the GUI stops sending commands (frozen, crashed tick), the arm slows to a stop.

All limits and the hard speed cap are still enforced by SafeArm.
"""

import math
import threading
import time

from arm_driver import MIN_WRITE_GAP_S, ArmSafetyError


def _approach(value, goal, max_delta):
    if value < goal:
        return min(goal, value + max_delta)
    return max(goal, value - max_delta)


class MotionStreamer:
    def __init__(self, arm, arm_cfg, clock=time.monotonic):
        self.arm = arm
        self.clock = clock
        self.period = float(arm_cfg.get("stream_period_ms", 40)) / 1000.0
        self.lookahead = float(arm_cfg.get("lookahead_ms", 120)) / 1000.0
        self.accel = float(arm_cfg.get("accel_units_per_s2", 500))
        self.decel = float(arm_cfg.get("decel_units_per_s2", 1000))
        self.command_timeout = float(arm_cfg.get("command_timeout_s", 0.25))
        if self.period < MIN_WRITE_GAP_S:
            raise ArmSafetyError(f"stream_period_ms must be at least {MIN_WRITE_GAP_S * 1000:.0f}")
        if not self.period <= self.lookahead <= 0.5:
            raise ArmSafetyError("lookahead_ms must be between stream_period_ms and 500")
        if self.accel <= 0 or self.decel <= 0:
            raise ArmSafetyError("accel/decel must be positive")

        self._goal_lock = threading.Lock()  # guards _goal / _goal_t (cheap, GUI-facing)
        self._step_lock = threading.Lock()  # guards velocities and all streaming I/O
        self._goal = {}
        self._goal_t = -math.inf
        self._vel = {}
        self.error = None
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------ GUI side
    def command(self, velocities, now=None):
        """Set the desired signed velocity (units/s) per servo. Servos left
        out slow down to a stop. Call this regularly; see command_timeout_s."""
        cap = self.arm.hard_max_speed
        goal = {int(s): max(-cap, min(cap, float(v))) for s, v in velocities.items() if v}
        with self._goal_lock:
            self._goal = goal
            self._goal_t = self.clock() if now is None else now

    def halt(self):
        """Stop at once (emergency stop / disarm): every moving servo is told to
        stay at its current setpoint."""
        with self._goal_lock:
            self._goal = {}
        with self._step_lock:
            moving = list(self._vel)
            self._vel.clear()
            if moving and self.error is None:
                self.arm.hold(moving)

    def is_moving(self):
        return bool(self._vel)

    def clear_error(self):
        self.error = None

    # ------------------------------------------------------------- stepping
    def step(self, dt, now=None):
        """One streaming cycle. Runs in the thread; tests call it directly."""
        now = self.clock() if now is None else now
        dt = min(max(dt, 0.0), 0.1)  # a stalled thread must never cause a jump
        with self._goal_lock:
            goal = dict(self._goal) if now - self._goal_t <= self.command_timeout else {}
        with self._step_lock:
            if self.error is not None:
                return
            setpoints, leads = {}, {}
            for sid in set(goal) | set(self._vel):
                v = self._vel.get(sid, 0.0)
                g = goal.get(sid, 0.0)
                speeding_up = g * v >= 0 and abs(g) > abs(v)
                v = _approach(v, g, (self.accel if speeding_up else self.decel) * dt)
                if v == 0.0 and sid not in self._vel:
                    continue  # idle and staying idle
                lo, hi = self.arm.limits[sid]
                p = self.arm.setpoint(sid) + v * dt
                if (p >= hi and v > 0) or (p <= lo and v < 0):
                    p, v = min(hi, max(lo, p)), 0.0  # reached a soft limit
                setpoints[sid] = p
                leads[sid] = v * self.lookahead
                if v == 0.0:
                    self._vel.pop(sid, None)  # this packet is its final "stop here"
                else:
                    self._vel[sid] = v
            if not setpoints:
                return
            try:
                self.arm.stream(setpoints, leads, self.lookahead * 1000.0)
            except Exception as exc:
                self.error = exc
                self._vel.clear()
                with self._goal_lock:
                    self._goal = {}

    # --------------------------------------------------------------- thread
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="arm-motion", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        try:
            self.halt()
        except Exception:
            pass

    def _run(self):
        last = next_t = time.monotonic()
        while not self._stop.is_set():
            next_t += self.period
            wait = next_t - time.monotonic()
            if wait > 0:
                if self._stop.wait(wait):
                    break
            else:
                next_t = time.monotonic()  # fell behind: don't try to catch up
            now = time.monotonic()
            self.step(now - last)
            last = now
