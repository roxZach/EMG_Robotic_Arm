"""Read-only xArm check: battery voltage and servo positions vs. config limits.

    python code/check_arm.py                 # read only, the arm does not move
    python code/check_arm.py --nudge 6       # move servo 6 by +30 units and back, slowly
    python code/check_arm.py --glide 6       # smooth streamed move +80 and back (tests smoothness)

Use --nudge on each joint (6, 5, 4) to confirm which way "+" goes before
setting the directions in config.json.
"""

import argparse
import sys
import time

from arm_driver import SafeArm, XArmBackend, load_config
from motion import MotionStreamer

NUDGE = 30
GLIDE = 80
GLIDE_SPEED = 60  # units/s


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    parser.add_argument("--nudge", type=int, metavar="SERVO", help="servo id to move +30 units and back")
    parser.add_argument("--glide", type=int, metavar="SERVO",
                        help="servo id to glide +80 units and back with the smooth motion streamer")
    args = parser.parse_args()

    config = load_config(args.config)
    try:
        backend = XArmBackend(config["arm"].get("connection", "USB"))
    except ImportError as exc:
        sys.exit(f"Missing library ({exc}). Run: pip install -r requirements.txt")
    except Exception as exc:
        sys.exit(f"Could not open the arm: {exc}\nIs it switched on, plugged in by USB, and not used by another program?")
    arm = SafeArm(config, backend)

    ok, voltage = arm.battery()
    print(f"Supply voltage: {voltage if voltage else 'n/a'} V  ->  {'OK' if ok else 'TOO LOW - charge / use the 7.5 V adapter'}")
    positions = arm.read_positions()
    print("\nServo  name           position   limits      status")
    for sid in sorted(arm.limits):
        lo, hi = arm.limits[sid]
        pos = positions[sid]
        state = "unreadable" if pos is None else ("ok" if lo <= pos <= hi else "OUTSIDE LIMITS (Go Home will fix)")
        print(f"  {sid}    {arm.names[sid]:<13}  {str(pos):>8}   {lo:>4}-{hi:<4}   {state}")

    if args.nudge is not None:
        sid = args.nudge
        pos = positions.get(sid)
        if sid not in arm.limits or pos is None:
            sys.exit(f"Servo {sid}: unknown limits or position, not moving.")
        lo, hi = arm.limits[sid]
        if not lo <= pos + NUDGE <= hi:
            sys.exit(f"Servo {sid}: +{NUDGE} from {pos} would leave {lo}-{hi}. Go Home first (in the app).")
        if not ok:
            sys.exit("Voltage too low, not moving.")
        if input(f"\nServo {sid} ({arm.names[sid]}) will move +{NUDGE} units and back. Area clear? [y/N] ").lower() != "y":
            return
        arm.homed = True  # we start from a measured, in-limit position
        arm._send(sid, pos + NUDGE, 1500)
        time.sleep(2.0)
        print(f"Moved to {pos + NUDGE}. Note which way it went: that is the '+' direction.")
        arm._send(sid, pos, 1500)
        time.sleep(1.6)
        print("Returned.")

    if args.glide is not None:
        glide(arm, positions, args.glide, ok, config)
    arm.close()


def glide(arm, positions, sid, ok, config):
    pos = positions.get(sid)
    if sid not in arm.limits or pos is None:
        sys.exit(f"Servo {sid}: unknown limits or position, not moving.")
    lo, hi = arm.limits[sid]
    direction = 1 if pos + GLIDE <= hi else -1
    if not lo <= pos + direction * GLIDE <= hi:
        sys.exit(f"Servo {sid}: not enough room for a {GLIDE}-unit glide inside {lo}-{hi}. Go Home first.")
    if not ok:
        sys.exit("Voltage too low, not moving.")
    if input(f"\nServo {sid} ({arm.names[sid]}) will glide {direction * GLIDE:+d} units and back at "
             f"{GLIDE_SPEED} units/s. Area clear? [y/N] ").lower() != "y":
        return
    arm.homed = True  # we start from a measured, in-limit position
    streamer = MotionStreamer(arm, config["arm"])
    streamer.start()
    try:
        for sign in (direction, -direction):
            goal = pos + direction * GLIDE if sign == direction else pos
            t0 = time.monotonic()
            while sign * (goal - arm.setpoint(sid)) > 2 and time.monotonic() - t0 < 5:
                if streamer.error:
                    sys.exit(f"Arm error: {streamer.error}")
                streamer.command({sid: sign * GLIDE_SPEED})
                time.sleep(0.02)
            streamer.command({})
            time.sleep(0.6)
            print(f"At {arm.setpoint(sid):.0f}.")
        print("Done. The motion should have been one continuous sweep each way, with no stutter.")
    finally:
        streamer.stop()


if __name__ == "__main__":
    main()
