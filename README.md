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

Movement starts at half speed and reaches full speed after 0.6 s of holding, for fine positioning. Starts and stops are smoothed by an acceleration limit.
Servos 2 and 3 (the wrist) are never moved by gestures. Servo 2 only goes to 500 when you press Go Home.

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

Now `code/motion.py` (`MotionStreamer`) runs in its own thread:

- The GUI only sets a target **velocity** per joint.
- Every 40 ms the thread sends **one** packet for all moving joints. Each joint is aimed 120 ms ahead of its setpoint with a 120 ms duration, so the servo moves at a constant speed and never stops between packets.
- Velocity ramps up and down with an acceleration limit (soft start / soft stop). On release the joint gets a final "stop here" command.
- E-stop / disarm stop at once. If the GUI stops sending commands for 0.25 s, the arm slows to a stop by itself.

## Safety features

- **Nothing moves until you press Go Home, then ARM.** Go Home asks for confirmation and is slow (≥3 s).
- **Soft limits** for every servo are taken from your vegetable-sorting code ([config.json](config.json)). Commands are clamped to them, and servos without limits are refused.
- **Speed cap.** Every command's duration is stretched so that no servo exceeds `hard_max_speed_units_per_s` (300 units/s ≈ 72°/s). The default speed is 100 units/s ≈ 24°/s.
- **No accidental motion from a misclassification.** A gesture must win 4 of the last 5 predictions, each with at least 65% confidence, to start. Once moving it continues while it holds 3 of 5 votes, and a blip shorter than 0.3 s does not restart the speed ramp.
- **Hold on signal loss.** If EMG data or predictions stop for 0.3 s, the arm holds still.
- **Stuck-gesture lockout.** If the same motion continues for 6 s, it stops until you relax.
- **The arm never drops.** On stop, disarm, disconnect or app close, the servos keep holding their pose. The code never calls `servoOff()`, which would cut motor power and let the arm fall.
- **Voltage check.** It is checked on connect and every 10 s. Below 6.6 V the arm is stopped (low voltage causes servo brown-outs and USB drop-outs).
- **Errors stop motion.** Any USB error or unplugged cable stops motion immediately.
- Every session is logged to `logs/session_*.csv` (predictions, motions, servo targets). You can use the logs in your thesis.

## Before you start: hardware safety checklist

1. **Power the arm from its own 7.5 V adapter or charged battery.** USB is only for data.
2. **Keep the arm's reach clear.** Place it on a stable table with nothing, and no hands, within about 40 cm. The person wearing the armband sits **outside** that reach.
3. **Electrical safety for the person wearing the electrodes.** Use the **Wi-Fi** connection for the armband, or if you use the USB cable, run the laptop **on battery (charger unplugged)**. The person wearing it should not touch the robot's metal parts or power supply.
4. **Only one program may control the arm.** Close the vegetable-sorting script first (it moves the arm home as soon as it starts).
5. **Know your physical stop.** The arm's power switch cuts everything, but the arm will then fall. Use Space/Esc first. Use the switch only if the software stop fails, and support the arm with one hand while you switch it off.
6. **Don't leave the gripper squeezing an object for long.** A blocked servo heats up. If needed, lower `gripper.close` in `config.json` so it grips firmly without straining.

## Setup (once)

```bash
cd ~/Documents/Study/Thesis
source .venv/bin/activate
pip install -r EMG_Robotic_Arm/requirements.txt
```

`xarm` and `hidapi` are already installed in `.venv`. The only remaining change in that file is upgrading
scikit-learn to 1.9: your models were saved with 1.9.0 and `.venv` has 1.8.0. They work, but sklearn prints a version warning.

## Bring-up, one stage at a time (do not skip stages)

Run the app from the `Thesis` folder:

```bash
python EMG_Robotic_Arm/code/emg_arm_control.py
```

**Stage 1 – no hardware.** Tick **Simulate arm** (it is ticked automatically when no xArm is plugged in).
Start `python ArmBand/code/emg_simulator_app.py` (TCP mode), load a model, choose
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

Use your best model. `06_aug_shimul_rf_training_…` (94% accuracy) or your own `rf_training_20260801_002832` (91%) are good candidates.
Retrain with your own recordings if you are the person wearing it. Do each gesture and confirm in the **Live**
panel that the right action appears and that **Rest** reliably shows `HOLD`. Raise *Min confidence* if you see false moves.

**Stage 3 – real arm, keyboard only.** Plug in the arm and switch it on, then run the read-only check:

```bash
cd EMG_Robotic_Arm
python code/check_arm.py            # prints voltage + positions, does not move
python code/check_arm.py --nudge 6  # base moves +30 units and back (asks first)
python code/check_arm.py --nudge 5  # shoulder
python code/check_arm.py --nudge 4  # elbow
python code/check_arm.py --glide 6  # smooth streamed sweep +80 and back: should look like one continuous motion
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
| `arm.accel_units_per_s2`, `decel_units_per_s2` | Softness of starts and stops (lower = softer, slower to respond) |
| `filter.hold_votes`, `dropout_grace_s` | Keep moving through brief misclassifications |
| `filter.switch_hold_s` | Fist hold time to switch shoulder/elbow (0 = off, the fist toggles the gripper at once) |
| `filter.ramp_start`, `ramp_time_s` | Speed at the start of a hold, and the time to reach full speed |
| `filter.min_confidence`, `vote_window`, `stable_votes` | How certain a gesture must be before the arm moves |
| `filter.grip_hold_s`, `grip_cooldown_s` | How long to hold a fist, and the minimum time between gripper toggles |
| `filter.max_continuous_move_s` | Stuck-gesture lockout |

The app validates the config at start-up and refuses unsafe values (e.g. a home pose outside the limits).

## Files

```
EMG_Robotic_Arm/
├── config.json             limits, home pose, gesture mapping, filter settings
├── requirements.txt
├── code/
│   ├── emg_arm_control.py  the app (GUI). Reuses the stream/inference code in ArmBand/code/mouse.py
│   ├── arm_driver.py       SafeArm: limits, speed cap, homing, gripper; real + simulated arm
│   ├── motion.py           MotionStreamer: smooth velocity streaming in its own thread
│   ├── teleop.py           label mapping, vote filter, timeouts, lockout, control tick
│   └── check_arm.py        read-only hardware check + single-joint nudge
├── tests/test_safety.py    34 safety + smoothness tests, no hardware needed
└── logs/                   session CSVs (created at run time)
```

Run the tests: `python -m unittest discover EMG_Robotic_Arm/tests`

## Troubleshooting

| Problem | Fix |
|---|---|
| Status says *SIMULATION ONLY* | The **Simulate arm** box was ticked, so the real arm won't move. Disconnect, untick it, and connect again. It is unticked automatically when the xArm is plugged in at start-up |
| `open failed` when connecting the arm | Arm switched off, USB cable is power-only, or another program has the arm open |
| `No module named 'hid'` | `pip install hidapi` (not the package named `hid`) |
| Arm never moves with EMG (Live panel shows the right gesture) | The confidence is below *Min confidence*. Look at the % in "Model output". If correct gestures sit at 50–60%, lower *Min confidence* a little, or retrain with fresh recordings of yourself |
| Motion still a bit jerky on the real arm | Raise `arm.lookahead_ms` to 160–200 in `config.json`. Check with `check_arm.py --glide 6` |
| Arm moves when you are relaxed | Raise *Min confidence*, re-record Rest, retrain, and make sure the electrodes sit on the same spot as when you recorded |
| "Same gesture too long – relax" | Lockout after 6 s of continuous motion. Relax briefly to continue |
| "… at its safe limit" | The joint reached its soft limit. Move the other way |
| Arm stops by itself | Look at the yellow note: EMG lost, low voltage, or a USB error |
