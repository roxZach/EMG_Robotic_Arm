# EMG Robotic Arm Control

Control the Hiwonder **xArm 1S** with gestures from the **BioWave ESP32-S3 EMG armband**.
Both devices connect to the laptop. The app reads EMG and classifies it with your trained Random Forest. It turns the gesture into a joint velocity, and a separate motion thread streams smooth, speed-limited moves to the arm.

```
Armband ──Wi-Fi/USB──▶ laptop: RF model ─▶ vote filter ─▶ velocities ─▶ motion thread ─▶ safety layer ─▶ USB ──▶ xArm 1S
```

## Gestures

| Gesture (model class) | Arm action |
|---|---|
| `Left` / `Right` | Rotate base (servo 6) |
| `up`, `Move_Up` / `down`, `Move_Down` | Move the selected joint: **Shoulder** (servo 5) or **Elbow** (servo 4) |
| `fist_close` held ≈0.6 s, then **relax** | Toggle gripper open/closed (servo 1) |
| `fist_close` held ≈1.8 s | **Switch** Up/Down between shoulder and elbow (the gripper does not toggle) |
| `Thumbs_Up` / `Fist_Open_Fast` (if the model has one) | Switch joint. The fist then toggles the gripper as soon as it has been held 0.6 s |
| `Rest` | Hold position |

With these you can do a whole pick-and-place with EMG only: rotate the base, lower the shoulder, switch to the elbow, close the gripper, lift, rotate and open.
The Live panel shows which joint Up/Down moves now, and guides you while you hold a fist.

Movement starts at half speed and reaches full speed after 0.6 s of holding, for fine positioning. Starts, stops and direction changes follow an S-curve (limited acceleration and jerk), and a joint slows down before its soft limit instead of stopping dead.
Servos 2 and 3 (the wrist) are never moved by gestures. Servo 2 only goes to 500 when you press Go Home.

### Change the gesture mapping

The table above is the default, based on each class's name. To change it, load a model, make sure the arm is disarmed, and click **Gesture Mapping**. It lists the gestures the model was trained on, with the number of training windows and the test accuracy of each (from `training_results.json`, if it is next to the model; below 70% is shown in red). Pick an action for each gesture and click **Save Mapping**.

The mapping takes effect at once and is saved as `gesture_mapping.json` next to the model, so each model keeps its own. At least one gesture must be set to Rest. **Reset to Defaults** goes back to the name-based mapping. To change which joint an action moves, edit `gestures` in `config.json`.

**Keyboard** (works only while ARMED; **keys can be held together**, e.g. base + shoulder):

| Key | Action |
|---|---|
| `←` `→` | Base |
| `↑` `↓` | Selected joint (`1` = shoulder, `2` = elbow), same as the gestures |
| `W` `S` | Shoulder up / down |
| `E` `D` | Elbow up / down |
| `G` | Toggle gripper |
| **`Space` / `Esc`** | **EMERGENCY STOP** (works at any time) |

## Smooth motion (how it works)

The servos take "go to P within T ms" commands, and every new command restarts the move. The old code sent a tiny step from the GUI timer every 50 ms and did the USB write (and a 30 ms wait) on the GUI thread. Motion stuttered and lagged whenever the GUI was busy.

Now `emg_arm/arm/motion.py` (`MotionStreamer`) runs in its own thread:

- The GUI only sets a target **velocity** per joint.
- Every 40 ms the thread sends **one** packet for all moving joints. Each joint is aimed 120 ms ahead of its setpoint with a 120 ms duration, so the servo moves at a constant speed and never stops between packets.
- Velocity ramps up and down along an S-curve: acceleration is limited, and so is how fast the acceleration changes (jerk). There is no sudden kick at a start, stop or reversal, which keeps the plastic arm from wobbling. On release the joint gets a final "stop here" command.
- Near a soft limit the joint never goes faster than it can stop in the room left, so it glides into the limit.
- E-stop / disarm stop at once. If the GUI stops sending commands for 0.25 s, the arm slows to a stop by itself.

## Safety features

- **Nothing moves until you press Go Home, then ARM.** Go Home asks for confirmation and is slow (≥3 s).
- **Soft limits** for every servo are taken from your vegetable-sorting code ([config.json](config.json)). Commands are clamped to them, and servos without limits are refused.
- **Speed cap.** Every command's duration is stretched so that no servo exceeds `hard_max_speed_units_per_s` (300 units/s ≈ 72°/s). The default speed is 100 units/s ≈ 24°/s.
- **No accidental motion from a misclassification.** A gesture must win 4 of the last 5 predictions, each with at least 65% confidence, to start. Once moving it continues while it holds 3 of 5 votes, and a blip shorter than 0.3 s does not restart the speed ramp.
- **Hold on signal loss.** If EMG data or predictions stop for 0.3 s, the arm holds still.
- **Stuck-gesture lockout.** If the same motion continues for 6 s, it stops until you relax.
- **Parks itself when the app closes.** Closing the window, Cmd+Q, Ctrl+C in the terminal, `kill`, or closing the terminal all do the same thing:
  1. The arm moves slowly (≥ 3 s) to the **home pose**.
  2. It **opens the gripper**, so the servo isn't left squeezing. Anything still held drops from the home pose.
  3. It **reads the servos back** to confirm the arm really is at home. If it isn't, a warning names the joint.
  4. Only then does the app exit.

  The status shows *PARKING ARM, THEN CLOSING* while this runs. **Space/Esc stops the parking move** where the arm is, and the app stays open. Closing again asks whether to park or to exit without parking.
  Turn this off with `arm.park_on_exit` / `arm.open_gripper_on_exit` in `config.json`.
  If the app is force-killed (`kill -9`, Activity Monitor → Force Quit) or the laptop loses power, it cannot park. The arm then just holds its pose.
- **Space/Esc also stops Go Home** in the middle of the move. The app reads where each joint is and holds it there.
- **The arm never drops.** On stop, disarm, disconnect or app close, the servos keep holding their pose. The code never calls `servoOff()`, which would cut motor power and let the arm fall.
- **An unexpected software error doesn't crash the app.** It stops motion and shows the error, so you can still close the app normally (and park).
- **Voltage check.** It is checked on connect and every 10 s. Below 6.6 V the arm is stopped (low voltage causes servo brown-outs and USB drop-outs).
- **Errors stop motion.** Any USB error or unplugged cable stops motion immediately.
- Every session is logged to `logs/session_*.csv` (predictions, motions, servo targets). You can use the logs in your thesis.

## Before you start: hardware safety checklist

1. **Power the arm from its own 7.5 V adapter or charged battery.** USB is only for data.
2. **Keep the arm's reach clear.** Place it on a stable table with nothing, and no hands, within about 40 cm. The person wearing the armband sits **outside** that reach.
3. **Electrical safety for the person wearing the electrodes.** Use the **Wi-Fi** connection for the armband, or if you use the USB cable, run the laptop **on battery (charger unplugged)**. The person wearing it should not touch the robot's metal parts or power supply.
4. **Only one program may control the arm.** Close the vegetable-sorting script first (it moves the arm home as soon as it starts).
5. **Know your physical stop.** The arm's power switch cuts everything, but the arm will then fall. Use Space/Esc first. Use the switch only if the software stop fails, and support the arm with one hand while you switch it off.
6. **Before closing the app, put down anything the gripper holds.** Parking opens the gripper at the home pose.
7. **Don't leave the gripper squeezing an object for long.** A blocked servo heats up. If needed, lower `gripper.close` in `config.json` so it grips firmly without straining.

## Quick start (clone and run)

This repo is self-contained: the code, two trained models (`models/`) and the setup logic are all in it.
You only need **Python 3.11–3.13** ([python.org](https://www.python.org/downloads/)) and, on the first run, internet.

```bash
git clone https://github.com/roxZach/EMG_Robotic_Arm.git
cd EMG_Robotic_Arm
python3 main.py        # Windows: py main.py
```

There is no `pip install` step and no venv to activate.
- **First run:** the script creates `.venv/` inside this folder, installs the pinned libraries from `requirements.txt` (about 1–3 minutes), then starts.
- **Later runs:** start immediately. If `requirements.txt` changes after a `git pull`, the next run updates `.venv` by itself.
- **Any Python works to start it.** You can use `python3`, the VS Code Run button, or an IDE. The script restarts itself inside `.venv` ([emg_arm/env_bootstrap.py](emg_arm/env_bootstrap.py)).

Platform notes:
- **macOS / Windows:** the xArm works over USB with no driver.
- **Linux:** to open the xArm without `sudo`, add a udev rule once:
  `echo 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5750", MODE="0666"' | sudo tee /etc/udev/rules.d/99-xarm.rules && sudo udevadm control --reload-rules`, then replug the arm.
  If `.venv` cannot be created, run `sudo apt install python3-venv`.

## Bring-up, one stage at a time (do not skip stages)

Run the app from this folder:

```bash
python3 main.py
```

**Stage 1 – no hardware.** Tick **Simulate arm** (it is ticked automatically when no xArm is plugged in).
Start `python3 tools/emg_simulator.py` (TCP mode), load a model, choose
*Wired Serial / Simulator → Simulator (local)*, and click **Connect Armband**. Then click **Connect Arm**, **Go Home** and **ARM**.
Watch the servo bars move, and practise the E-STOP.

**Stage 2 – real armband, simulated arm.** Load your model, then connect the armband over Wi-Fi.
This uses the same flow as the mouse controller:

1. **USB Provision.** Plug the ESP32 into the laptop by USB and click **USB Provision**. Pick its serial port and click **Probe Selected Port**.
   The dialog should show the device name, ID and firmware. Then enter your Wi-Fi name and password and click **Send Credentials**.
   (You only need to do this once per Wi-Fi network. The ESP32 remembers it.)
2. **Same Wi-Fi.** The ESP32 restarts and joins that Wi-Fi. The laptop must be on the **same** Wi-Fi (turn off any VPN).
   You can unplug the USB cable now and power the armband from its battery.
3. **Discover.** The app searches automatically for about 20 s after provisioning and selects the ESP32 it finds (shown as `name @ IP`).
   On later sessions, just click **Discover**. If nothing shows up, type the ESP32's IP address into the box.
4. **Connect Armband.** The access key must match the one in the firmware.

**Load RF Model** opens `models/`, which contains two bundled models: `06_aug_shimul_rf_training_…` (94% test accuracy) and `rf_training_20260801_002832` (91%).
EMG differs a lot between people, so a model trained on someone else will be weaker for you. To add your own, copy its training folder (with `rf_realtime_model.joblib`) into `models/`.
Retrain with your own recordings if you are the person wearing it. Do each gesture and confirm in the **Live**
panel that the right action appears and that **Rest** reliably shows `HOLD`. Raise *Min confidence* if you see false moves.

**Stage 3 – real arm, keyboard only.** Plug in the arm and switch it on, then run the read-only check:

```bash
python3 tools/check_arm.py            # prints voltage + positions, does not move
python3 tools/check_arm.py --nudge 6  # base moves +30 units and back (asks first)
python3 tools/check_arm.py --nudge 5  # shoulder
python3 tools/check_arm.py --nudge 4  # elbow
python3 tools/check_arm.py --glide 6  # smooth streamed sweep +80 and back: should look like one continuous motion
```

Use the nudges to confirm which way "+" goes. If Left/Right or Up/Down turn out reversed, flip the sign in `config.json`
(`gestures.left/right.direction` or `vertical_joints.*.up_direction`).
Then, in the app: untick **Simulate arm** → **Connect Arm** → **Go Home** → untick **Use EMG gestures** → **ARM**.
Drive each joint with the arrow keys at speed 50 to each end of its range. If any limit lets the arm hit the table or itself,
make that limit tighter in `config.json` and restart the app.

**Stage 4 – armband + real arm.** Tick **Use EMG gestures**, keep the speed at 50–100, ARM, and start with small
movements. Keep a finger near the Space bar.

## Tuning ([config.json](config.json))

| Setting | Meaning |
|---|---|
| `arm.servos.N.min/max` | Soft limits (xArm units 0–1000, ≈0.24°/unit) |
| `arm.home` | Home pose (must be inside the limits; checked at start-up) |
| `arm.gripper.open/close` | Gripper positions |
| `arm.default_speed_units_per_s` / `hard_max_speed_units_per_s` | Start speed / absolute speed cap |
| `arm.stream_period_ms`, `lookahead_ms` | Motion thread rate (≥ 30 ms) and how far ahead each joint is aimed. If motion still stutters on the real arm, raise `lookahead_ms` to 160–200 |
| `arm.accel_units_per_s2`, `decel_units_per_s2`, `jerk_units_per_s3` | Softness of starts and stops (lower = softer, slower to respond). With the defaults a 100 u/s joint stops in about 0.25 s / 9 units (≈2°) |
| `filter.hold_votes`, `dropout_grace_s` | Keep moving through brief misclassifications |
| `filter.switch_hold_s` | Fist hold time to switch shoulder/elbow (0 = off, the fist toggles the gripper at once) |
| `filter.ramp_start`, `ramp_time_s` | Speed at the start of a hold, and the time to reach full speed |
| `filter.min_confidence`, `vote_window`, `stable_votes` | How certain a gesture must be before the arm moves |
| `filter.grip_hold_s`, `grip_cooldown_s` | How long to hold a fist, and the minimum time between gripper toggles |
| `filter.max_continuous_move_s` | Stuck-gesture lockout |

The app validates the config at start-up and refuses unsafe values (e.g. a home pose outside the limits).

## Files

Start with `main.py`: it is the only file you run for the app, and its top comment is a map of the code.

```
EMG_Robotic_Arm/
├── main.py                     ▶ run this: starts the app
├── config.json                 limits, home pose, which joint each gesture moves, filter settings
├── requirements.txt
├── emg_arm/                    the app's code, grouped by job
│   ├── armband/                talk to the BioWave armband and recognise gestures
│   │   ├── link.py             Wi-Fi/USB link, USB provisioning, RF inference thread
│   │   └── features.py         EMG feature extraction (must match the one used in training)
│   ├── arm/                    move the xArm 1S safely
│   │   ├── gesture_control.py  gestures/keys → joint velocities: label mapping, vote filter, timeouts, lockout
│   │   ├── motion.py           MotionStreamer: smooth velocity streaming in its own thread
│   │   └── driver.py           SafeArm: limits, speed cap, homing, gripper; real + simulated arm
│   ├── app/                    the windows you see
│   │   ├── main_window.py      main window
│   │   ├── gesture_mapping.py  Gesture Mapping window
│   │   └── theme.py            colours and styles
│   └── env_bootstrap.py        creates/uses ./.venv automatically
├── tools/                      separate helper programs
│   ├── check_arm.py            read-only hardware check + single-joint nudge
│   └── emg_simulator.py        fake EMG stream for testing without the armband
├── tests/                      safety, smoothness, exit and gesture-mapping tests, no hardware needed
├── models/                     bundled Random Forest models (+ their training reports)
├── logs/                       session CSVs (created at run time, not in git)
└── .venv/                      Python environment (created on first run, not in git)
```

How data flows through the code:

```
armband ─▶ armband/link.py ─▶ armband/features.py + model ─▶ arm/gesture_control.py ─▶ arm/motion.py ─▶ arm/driver.py ─▶ xArm
```

`armband/link.py`, `armband/features.py`, `app/theme.py` and `tools/emg_simulator.py` were copied from the ArmBand project (2026-10-25) so this repo doesn't depend on it.
If you change the armband firmware protocol or the feature extraction there, copy the change here too.

Run the tests (they need no extra libraries): `python3 -m unittest discover tests`

## Troubleshooting

| Problem | Fix |
|---|---|
| Status says *SIMULATION ONLY* | The **Simulate arm** box was ticked, so the real arm won't move. Disconnect, untick it, and connect again. It is unticked automatically when the xArm is plugged in at start-up |
| `open failed` when connecting the arm | Arm switched off, USB cable is power-only, or another program has the arm open |
| `[setup] Installing … failed` | Usually no internet, or a Python version without ready-made packages. Use Python 3.12, delete the `.venv` folder and run again |
| `[setup] Python … is too old` | Install Python 3.11–3.13 from python.org (macOS's built-in `/usr/bin/python3` is 3.9) |
| Anything odd with libraries | Delete the `.venv` folder. The next run rebuilds it |
| Arm never moves with EMG (Live panel shows the right gesture) | The confidence is below *Min confidence*. Look at the % in "Model output". If correct gestures sit at 50–60%, lower *Min confidence* a little, or retrain with fresh recordings of yourself |
| Motion still a bit jerky on the real arm | Raise `arm.lookahead_ms` to 160–200 in `config.json`. Check with `tools/check_arm.py --glide 6` |
| Arm moves when you are relaxed | Raise *Min confidence*, re-record Rest, retrain, and make sure the electrodes sit on the same spot as when you recorded |
| "Same gesture too long – relax" | Lockout after 6 s of continuous motion. Relax briefly to continue |
| "… at its safe limit" | The joint reached its soft limit. Move the other way |
| Arm stops by itself | Look at the yellow note: EMG lost, low voltage, or a USB error |
