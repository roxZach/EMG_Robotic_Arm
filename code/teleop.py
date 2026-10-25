"""Turns noisy gesture predictions and held keys into safe arm motion.

Pure Python (no Qt) so the safety logic can be unit-tested.
"""

import re
from collections import Counter, deque

MOTIONS = ("left", "right", "up", "down")


def canonical_gesture(label):
    """Map model class names (Left, Move_Up, fist_close, Thumbs_Up, ...) to one of
    left/right/up/down/grip/switch/rest, or None for anything unrecognised."""
    words = set(re.sub(r"[^a-z]+", " ", str(label).lower()).split())
    if words & {"rest", "relax", "idle", "neutral"}:
        return "rest"
    # Checked before fist/up so "Fist_Open_Fast" and "Thumbs_Up" are not read
    # as grip / up.
    if words & {"thumb", "thumbs", "open", "spread", "switch", "mode"}:
        return "switch"
    if words & {"fist", "grip", "grab", "close", "closed"}:
        return "grip"
    for name in MOTIONS:
        if name in words:
            return name
    return None


class GestureFilter:
    """Majority vote with hysteresis + confidence gate + hold timers +
    stale/lockout guards.

    Fist (grip) gesture:
      * switch_hold_s > 0: hold >= grip_hold_s and relax  -> toggle gripper,
                           hold >= switch_hold_s          -> switch Up/Down joint.
      * switch_hold_s = 0: hold >= grip_hold_s -> toggle gripper at once.
    A dedicated "switch" class (e.g. Thumbs_Up) held >= grip_hold_s also switches.
    """

    def __init__(self, cfg):
        self.min_conf = float(cfg["min_confidence"])
        self.vote_window = int(cfg["vote_window"])
        self.stable_votes = int(cfg["stable_votes"])
        self.hold_votes = min(int(cfg.get("hold_votes", self.stable_votes)), self.stable_votes)
        self.grip_hold_s = float(cfg["grip_hold_s"])
        self.grip_cooldown_s = float(cfg["grip_cooldown_s"])
        self.switch_hold_s = float(cfg.get("switch_hold_s", 0.0))
        self.grace_s = float(cfg.get("dropout_grace_s", 0.0))
        self.timeout_s = float(cfg["prediction_timeout_s"])
        self.max_move_s = float(cfg["max_continuous_move_s"])
        self.reset()

    def reset(self):
        self.history = deque(maxlen=self.vote_window)
        self.last_prediction_t = None
        self.stable = None
        self.stable_since = 0.0
        self.grip_fired = False
        self.switch_fired = False
        self.grip_pending = False
        self.last_grip_t = float("-inf")
        self.last_switch_t = float("-inf")
        self.locked_out = False
        self._dropped = None  # (motion, since, locked_out, when) for the flicker grace

    def add_prediction(self, label, confidence, now):
        gesture = canonical_gesture(label) if confidence >= self.min_conf else None
        self.history.append(gesture)
        self.last_prediction_t = now
        counts = Counter(g for g in self.history if g is not None)
        if self.stable is not None and counts[self.stable] >= self.hold_votes:
            stable = self.stable  # hysteresis: easier to stay than to enter
        else:
            best = counts.most_common(1)
            stable = best[0][0] if best and best[0][1] >= self.stable_votes else None
        if stable != self.stable:
            self._change(stable, now)
        return gesture

    def _change(self, new, now):
        old = self.stable
        if old == "grip" and self.switch_hold_s > 0 and not self.switch_fired \
                and now - self.stable_since >= self.grip_hold_s:
            self.grip_pending = True  # fist released: toggle the gripper now
        since, locked = now, False
        dropped = self._dropped
        self._dropped = None
        if old in MOTIONS and new is None:
            self._dropped = (old, self.stable_since, self.locked_out, now)
        elif dropped and new == dropped[0] and now - dropped[3] <= self.grace_s:
            # A brief misclassification inside a held motion: carry on with the
            # same speed ramp and lockout timer instead of starting over.
            since, locked = dropped[1], dropped[2]
        self.stable, self.stable_since, self.locked_out = new, since, locked
        self.grip_fired = False
        self.switch_fired = False

    def is_fresh(self, now):
        return self.last_prediction_t is not None and now - self.last_prediction_t <= self.timeout_s

    def held_for(self, now):
        return max(0.0, now - self.stable_since)

    def motion(self, now):
        if not self.is_fresh(now) or self.stable not in MOTIONS or self.locked_out:
            return None
        if self.held_for(now) > self.max_move_s:
            # Same motion for too long usually means a stuck classification.
            # Require the gesture to change (e.g. rest) before moving again.
            self.locked_out = True
            return None
        return self.stable

    def take_grip_toggle(self, now):
        if not self.is_fresh(now):
            return False
        if self.switch_hold_s > 0:
            fire, self.grip_pending = self.grip_pending, False
        else:
            fire = self.stable == "grip" and not self.grip_fired and self.held_for(now) >= self.grip_hold_s
            if fire:
                self.grip_fired = True
        if not fire or now - self.last_grip_t < self.grip_cooldown_s:
            return False
        self.last_grip_t = now
        return True

    def take_switch(self, now):
        if not self.is_fresh(now) or self.switch_fired:
            return False
        held = self.held_for(now)
        fire = (self.stable == "switch" and held >= self.grip_hold_s) or \
               (self.stable == "grip" and self.switch_hold_s > 0 and held >= self.switch_hold_s)
        if not fire or now - self.last_switch_t < self.grip_cooldown_s:
            return False
        self.switch_fired = True
        self.last_switch_t = now
        return True

    def grip_hint(self, now):
        """Short guidance while a fist is held, for the Live panel."""
        if self.stable != "grip" or not self.is_fresh(now) or self.switch_fired:
            return ""
        held = self.held_for(now)
        if self.switch_hold_s <= 0:
            return "" if self.grip_fired else f"Fist {held:.1f}/{self.grip_hold_s:.1f} s → gripper"
        if held < self.grip_hold_s:
            return f"Fist {held:.1f} s…"
        return f"Fist {held:.1f} s: RELAX now = gripper, keep holding to {self.switch_hold_s:.1f} s = switch joint"


class Teleop:
    """One control tick: choose a motion source, turn it into per-servo
    velocities and hand them to the MotionStreamer."""

    def __init__(self, arm, gesture_filter, config, streamer):
        self.arm = arm
        self.filter = gesture_filter
        self.streamer = streamer
        self.gesture_map = config["gestures"]
        self.vertical_joints = config["vertical_joints"]
        self.vertical = next(iter(self.vertical_joints))
        fcfg = config["filter"]
        self.ramp_start = float(fcfg["ramp_start"])
        self.ramp_time = float(fcfg["ramp_time_s"])
        self.max_move_s = float(fcfg["max_continuous_move_s"])
        self.armed = False

    def servo_for(self, motion):
        """left/right/up/down (gestures; up/down use the selected joint) or
        '<joint>_up' / '<joint>_down' (direct keyboard keys)."""
        if motion in self.gesture_map:
            entry = self.gesture_map[motion]
            direction = int(entry["direction"])
            servo = entry["servo"]
            if servo != "vertical":
                return int(servo), direction
            joint = self.vertical_joints[self.vertical]
        else:
            name, _, way = motion.rpartition("_")
            joint = self.vertical_joints[name]
            direction = 1 if way == "up" else -1
        return int(joint["servo"]), direction * int(joint["up_direction"])

    def switch_vertical(self):
        names = list(self.vertical_joints)
        self.vertical = names[(names.index(self.vertical) + 1) % len(names)]
        return self.vertical

    def ramp(self, held_s):
        if self.ramp_time <= 0:
            return 1.0
        return self.ramp_start + (1.0 - self.ramp_start) * min(1.0, held_s / self.ramp_time)

    def tick(self, now, key_motions=None, key_grip=False, emg_ok=True):
        """key_motions: {motion: seconds held} for every motion key held.
        Returns a status dict for the UI. Raises on hardware errors."""
        status = {"motion": None, "source": None, "gripper": None, "switched": None,
                  "note": "", "velocities": {}}
        if not self.armed:
            self.streamer.command({}, now)
            return status

        moves, grip, switch = {}, False, False
        if key_motions or key_grip:
            source = "keyboard"
            moves = {m: h for m, h in (key_motions or {}).items() if h <= self.max_move_s}
            if len(moves) < len(key_motions or {}):
                status["note"] = "Key held too long - release it"
            grip = key_grip
        elif emg_ok:
            source = "emg"
            motion = self.filter.motion(now)
            if motion:
                moves = {motion: self.filter.held_for(now)}
            grip = self.filter.take_grip_toggle(now)
            switch = self.filter.take_switch(now)
            if self.filter.locked_out:
                status["note"] = "Same gesture too long - relax to continue"
        else:
            self.streamer.command({}, now)
            status["note"] = "EMG stream lost - holding position"
            return status

        if switch:
            status["switched"] = self.switch_vertical()
            status["source"] = source
        if grip:
            status["gripper"] = "closed" if self.arm.toggle_gripper() else "open"
            status["source"] = source

        velocities = {}
        for motion, held in moves.items():
            servo, direction = self.servo_for(motion)
            if self.arm.at_limit(servo, direction):
                status["note"] = f"{self.arm.names[servo]} at its safe limit - move the other way"
                continue
            v = velocities.get(servo, 0.0) + direction * self.arm.speed * self.ramp(held)
            velocities[servo] = max(-self.arm.speed, min(self.arm.speed, v))
        self.streamer.command(velocities, now)
        if moves:
            status["motion"] = " + ".join(sorted(moves))
            status["source"] = source
        status["velocities"] = velocities
        return status
