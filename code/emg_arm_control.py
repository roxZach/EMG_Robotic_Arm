"""BioWave EMG -> Hiwonder xArm 1S gesture control.

Loads a Random Forest model trained in ArmBand/BioWave, streams the armband
(Wi-Fi, USB serial, or the simulator), and maps gestures to arm motion:

    Left / Right   -> rotate base (servo 6)
    Up / Down      -> shoulder (servo 5) or elbow (servo 4), selectable
    Fist (hold)    -> hold ~0.6 s and relax: toggle gripper (servo 1)
                      keep holding ~1.8 s: switch Up/Down between shoulder/elbow
    Rest           -> hold position

Motion is streamed smoothly by motion.MotionStreamer in its own thread; the GUI
only decides velocities. The arm only moves while ARMED. Space or Esc is an
emergency stop at any time.

Closing the app (window close, Cmd+Q, Ctrl+C, kill) first parks the arm: it
moves slowly to the home pose, opens the gripper and is checked, then the app
exits. Space/Esc stops the parking move.
"""

import csv
import json
import os
import signal
import sys
import time
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))

import env_bootstrap  # noqa: E402  sets up / switches to the project's .venv if needed

env_bootstrap.ensure()

import warnings  # noqa: E402

import joblib
import numpy as np
import serial.tools.list_ports
import threading

from PyQt5.QtCore import QEvent, QObject, Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QDoubleSpinBox, QFileDialog, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QMessageBox, QProgressBar, QPushButton, QSpinBox, QStackedWidget,
    QVBoxLayout, QWidget,
)

from app_theme import apply_dark_theme, themed_button_style, themed_label_style
from armband_link import (
    DEFAULT_ACCESS_KEY, WIFI_STREAM_PORT, ControlProtocol, DeviceInfo,
    InferenceWorker, SerialWorker, USBProvisionDialog,
    WirelessStreamWorker, get_local_ip_for_target,
)

from arm_driver import ArmSafetyError, SafeArm, SimulatedBackend, XArmBackend, load_config
from motion import MotionStreamer
from teleop import GestureFilter, Teleop, canonical_gesture

try:
    # Models saved with a slightly different scikit-learn version still work; don't nag.
    from sklearn.exceptions import InconsistentVersionWarning
    warnings.filterwarnings("ignore", category=InconsistentVersionWarning)
except ImportError:
    pass

TRAINED_MODEL_DIR = PROJECT_ROOT / "models"
LOG_DIR = PROJECT_ROOT / "logs"
SAMPLE_RATE = 500


def console(message):
    """print() that never raises (the terminal may be gone, e.g. after SIGHUP)."""
    try:
        print(message, flush=True)
    except (OSError, ValueError):
        pass

# Several keys can be held at once (e.g. base + shoulder together).
KEY_MOTIONS = {Qt.Key_Left: "left", Qt.Key_Right: "right", Qt.Key_Up: "up", Qt.Key_Down: "down",
               Qt.Key_W: "shoulder_up", Qt.Key_S: "shoulder_down",
               Qt.Key_E: "elbow_up", Qt.Key_D: "elbow_down"}
KEY_NAMES = {Qt.Key_Left: "←", Qt.Key_Right: "→", Qt.Key_Up: "↑", Qt.Key_Down: "↓",
             Qt.Key_W: "W", Qt.Key_S: "S", Qt.Key_E: "E", Qt.Key_D: "D"}
STOP_KEYS = (Qt.Key_Space, Qt.Key_Escape)


def control_with_retry(fn, *args, attempts=3):
    """Retry an ESP32 control command when its UDP reply is lost.

    Measured on a real network: ~1 in 5 replies never arrive, and START needs
    two round trips, so a single try fails about a third of the time."""
    for attempt in range(1, attempts + 1):
        try:
            return fn(*args)
        except RuntimeError as exc:
            if "timed out" not in str(exc) or attempt == attempts:
                raise


def network_hint(target_ip):
    local_ip = get_local_ip_for_target(target_ip)
    same = local_ip.split(".")[:2] == target_ip.split(".")[:2]
    hint = f"\n\nThis laptop: {local_ip}\nArmband (ESP32): {target_ip}\n"
    if not same:
        hint += ("These look like DIFFERENT networks. Put the laptop on the same Wi-Fi as the armband "
                 "(or USB Provision the armband onto the laptop's Wi-Fi), then Discover again.")
    else:
        hint += "Same network. Press Connect again (Wi-Fi may be dropping packets), or Discover to refresh the IP."
    return hint


class RetryingKeepaliveWorker(QThread):
    """Like mouse.KeepaliveWorker, but retries lost replies and re-sends START
    when stream data stops (the ESP32 ends the stream after 5 s without PING)."""

    notice = pyqtSignal(str)

    def __init__(self, target_ip, access_key, data_age_fn):
        super().__init__()
        self.target_ip = target_ip
        self.access_key = access_key
        self.data_age = data_age_fn
        self._stop = threading.Event()

    def run(self):
        while not self._stop.wait(1.0):
            try:
                if self.data_age() > 2.0:
                    control_with_retry(ControlProtocol.start_stream, self.target_ip, self.access_key,
                                       get_local_ip_for_target(self.target_ip), str(WIFI_STREAM_PORT), attempts=2)
                    self.notice.emit("restarted")
                else:
                    control_with_retry(ControlProtocol.ping, self.target_ip, self.access_key, attempts=2)
            except Exception as exc:
                if not self._stop.is_set():
                    self.notice.emit(str(exc).splitlines()[0])

    def stop(self):
        self._stop.set()
        self.wait()


def xarm_plugged_in():
    """True if a Hiwonder xArm controller (USB HID 0483:5750) is connected."""
    try:
        import hid
        return any(d["vendor_id"] == 0x0483 and d["product_id"] == 0x5750 for d in hid.enumerate())
    except Exception:
        return False


class KeyboardFilter(QObject):
    """App-wide key handling so E-STOP works whichever widget has focus."""

    def __init__(self, window):
        super().__init__()
        self.window = window

    def eventFilter(self, obj, event):
        etype = event.type()
        if etype in (QEvent.KeyPress, QEvent.KeyRelease):
            armed = bool(self.window.teleop and self.window.teleop.armed)
            # Dialogs (e.g. USB Provision) and text boxes need Space/Esc while disarmed,
            # e.g. for a Wi-Fi name with spaces.
            if QApplication.activeWindow() is not self.window:
                return False
            if not armed and isinstance(QApplication.focusWidget(), QLineEdit):
                return False
            return self.window.handle_key(event, etype == QEvent.KeyPress)
        if etype == QEvent.WindowDeactivate:
            self.window.pressed_keys.clear()
        return False


class EmgArmWindow(QMainWindow):
    def __init__(self, config):
        super().__init__()
        self.config = config
        self.setWindowTitle("BioWave — EMG Robotic Arm Control")
        self.resize(1180, 820)

        # Armband / model state
        self.model_loaded = False
        self.emg_connected = False
        self.worker = None
        self.keepalive_worker = None
        self.current_wireless_device = None
        self.num_channels = 11
        self.window_samples = 100
        self.stride_samples = 25
        self.sample_count = 0
        self.data_buffer = None
        self.baseline = None
        self.last_batch_t = 0.0
        self.discovery_attempts_left = 0
        self.inference_worker = InferenceWorker(SAMPLE_RATE)
        self.inference_worker.prediction_ready.connect(self.on_prediction)
        self.inference_worker.start()

        # Arm state
        self.arm = None
        self.streamer = None
        self.gesture_filter = GestureFilter(config["filter"])
        self.teleop = None
        self.home_busy_until = 0.0
        self.parking = None          # None / "home" / "gripper" while parking before exit
        self.park_aborted = False
        self.close_ready = False
        self.pressed_keys = {}
        self.key_grip_pending = False
        self.last_motion_t = 0.0
        self.log_file = None
        self.log_writer = None

        self.build_ui()
        self.refresh_ports()
        self.update_controls()

        self.control_timer = QTimer(self)
        self.control_timer.setTimerType(Qt.PreciseTimer)
        self.control_timer.setInterval(int(config["arm"]["control_period_ms"]))
        self.control_timer.timeout.connect(self.control_tick)
        self.control_timer.start()

        self.battery_timer = QTimer(self)
        self.battery_timer.setInterval(10000)
        self.battery_timer.timeout.connect(self.check_battery)
        self.battery_timer.start()

    # ================================================================== UI
    def build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(10)

        title = QLabel("EMG ROBOTIC ARM CONTROL")
        title.setStyleSheet("font-size: 21px; font-weight: 800; letter-spacing: 1px;")
        root.addWidget(title)

        # ---- 1. Armband
        band = QGroupBox("1. Armband (EMG)")
        bl = QGridLayout(band)
        load = QPushButton("Load RF Model")
        load.setStyleSheet(themed_button_style("accent"))
        load.clicked.connect(self.load_model)
        self.model_label = QLabel("No model loaded")
        self.model_label.setStyleSheet(themed_label_style("muted"))
        bl.addWidget(load, 0, 0)
        bl.addWidget(self.model_label, 0, 1, 1, 3)

        self.connection_mode = QComboBox()
        self.connection_mode.addItem("Wireless Wi-Fi (ESP32)", "wireless")
        self.connection_mode.addItem("Wired Serial / Simulator", "serial")
        self.connection_mode.currentIndexChanged.connect(
            lambda: self.connection_stack.setCurrentIndex(self.connection_mode.currentIndex()))
        bl.addWidget(self.connection_mode, 1, 0)

        self.connection_stack = QStackedWidget()
        # Same Wi-Fi flow as the mouse controller:
        # USB Provision (probe port + send Wi-Fi) -> Discover on the same Wi-Fi -> Connect.
        wpage = QWidget()
        wv = QVBoxLayout(wpage)
        wv.setContentsMargins(0, 0, 0, 0)
        wv.setSpacing(6)
        row_dev = QHBoxLayout()
        row_dev.addWidget(QLabel("Hardware Device:"))
        self.wireless_combo = QComboBox()
        self.wireless_combo.setEditable(True)
        self.wireless_combo.setMinimumWidth(170)
        self.wireless_combo.setPlaceholderText("Select or enter IP (e.g. 192.168.1.100)")
        row_dev.addWidget(self.wireless_combo, 1)
        discover = QPushButton("Discover")
        discover.setStyleSheet(themed_button_style("muted"))
        discover.clicked.connect(self.discover_wireless_devices)
        row_dev.addWidget(discover)
        wv.addLayout(row_dev)
        row_key = QHBoxLayout()
        row_key.addWidget(QLabel("Access Key:"))
        self.access_key_input = QLineEdit(DEFAULT_ACCESS_KEY)
        self.access_key_input.setEchoMode(QLineEdit.Password)
        self.access_key_input.setPlaceholderText("ESP32 access key")
        row_key.addWidget(self.access_key_input, 1)
        usb_provision = QPushButton("USB Provision")
        usb_provision.setToolTip("Plug the ESP32 in by USB, probe its port, and send your Wi-Fi name/password")
        usb_provision.setStyleSheet(themed_button_style("accent"))
        usb_provision.clicked.connect(self.open_usb_provision_dialog)
        row_key.addWidget(usb_provision)
        wv.addLayout(row_key)
        self.connection_stack.addWidget(wpage)
        spage = QWidget()
        sl = QHBoxLayout(spage)
        sl.setContentsMargins(0, 0, 0, 0)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        sl.addWidget(self.port_combo, 1)
        refresh = QPushButton("Refresh")
        refresh.setStyleSheet(themed_button_style("muted"))
        refresh.clicked.connect(self.refresh_ports)
        sl.addWidget(refresh)
        self.connection_stack.addWidget(spage)
        bl.addWidget(self.connection_stack, 1, 1, 1, 2)

        self.emg_button = QPushButton("Connect Armband")
        self.emg_button.setStyleSheet(themed_button_style("success"))
        self.emg_button.clicked.connect(self.toggle_emg)
        bl.addWidget(self.emg_button, 1, 3)
        self.emg_status = QLabel("ARMBAND: OFFLINE")
        self.emg_status.setStyleSheet(themed_label_style("muted"))
        bl.addWidget(self.emg_status, 2, 0, 1, 4)
        root.addWidget(band)

        # ---- 2. Robotic arm
        armbox = QGroupBox("2. Robotic Arm (xArm 1S)")
        al = QHBoxLayout(armbox)
        self.simulate_check = QCheckBox("Simulate arm (no hardware)")
        # Real arm plugged in -> default to it; otherwise default to simulation.
        self.simulate_check.setChecked(not xarm_plugged_in())
        al.addWidget(self.simulate_check)
        self.arm_button = QPushButton("Connect Arm")
        self.arm_button.setStyleSheet(themed_button_style("success"))
        self.arm_button.clicked.connect(self.toggle_arm)
        al.addWidget(self.arm_button)
        self.home_button = QPushButton("Go Home (slow)")
        self.home_button.setStyleSheet(themed_button_style("accent"))
        self.home_button.clicked.connect(self.go_home)
        al.addWidget(self.home_button)
        self.grip_open_button = QPushButton("Open Gripper")
        self.grip_open_button.setStyleSheet(themed_button_style("muted"))
        self.grip_open_button.clicked.connect(lambda: self.manual_gripper(False))
        al.addWidget(self.grip_open_button)
        self.grip_close_button = QPushButton("Close Gripper")
        self.grip_close_button.setStyleSheet(themed_button_style("muted"))
        self.grip_close_button.clicked.connect(lambda: self.manual_gripper(True))
        al.addWidget(self.grip_close_button)
        self.arm_status = QLabel("ARM: OFFLINE")
        self.arm_status.setStyleSheet(themed_label_style("muted"))
        al.addWidget(self.arm_status, 1)
        root.addWidget(armbox)

        # ---- 3. Control + live view
        row = QHBoxLayout()
        ctrl = QGroupBox("3. Control")
        cl = QVBoxLayout(ctrl)
        self.arm_toggle = QPushButton("ARM (enable motion)")
        self.arm_toggle.setMinimumHeight(48)
        self.arm_toggle.setStyleSheet(themed_button_style("success"))
        self.arm_toggle.clicked.connect(self.toggle_armed)
        cl.addWidget(self.arm_toggle)
        estop = QPushButton("EMERGENCY STOP  (Space / Esc)")
        estop.setMinimumHeight(64)
        estop.setStyleSheet(themed_button_style("danger") + "QPushButton { font-size: 18px; font-weight: 800; }")
        estop.clicked.connect(lambda: self.emergency_stop("Stop button pressed"))
        cl.addWidget(estop)
        self.state_label = QLabel("DISARMED")
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-size: 22px; font-weight: 800; color: #94A3B8;")
        cl.addWidget(self.state_label)

        form = QGridLayout()
        self.emg_enable_check = QCheckBox("Use EMG gestures (off = keyboard only)")
        self.emg_enable_check.setChecked(True)
        form.addWidget(self.emg_enable_check, 0, 0, 1, 2)
        form.addWidget(QLabel("Up/Down moves"), 1, 0)
        self.vertical_combo = QComboBox()
        for name in self.config["vertical_joints"]:
            self.vertical_combo.addItem(name.capitalize(), name)
        self.vertical_combo.currentIndexChanged.connect(self.on_vertical_changed)
        form.addWidget(self.vertical_combo, 1, 1)
        form.addWidget(QLabel("Speed (units/s)"), 2, 0)
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(10, int(self.config["arm"]["hard_max_speed_units_per_s"]))
        self.speed_spin.setSingleStep(10)
        self.speed_spin.setValue(int(self.config["arm"]["default_speed_units_per_s"]))
        self.speed_spin.valueChanged.connect(lambda v: self.arm and self.arm.set_speed(v))
        form.addWidget(self.speed_spin, 2, 1)
        form.addWidget(QLabel("Min confidence"), 3, 0)
        self.conf_spin = QDoubleSpinBox()
        self.conf_spin.setRange(40, 99)
        self.conf_spin.setSuffix(" %")
        self.conf_spin.setValue(self.gesture_filter.min_conf * 100)
        self.conf_spin.valueChanged.connect(lambda v: setattr(self.gesture_filter, "min_conf", v / 100))
        form.addWidget(self.conf_spin, 3, 1)
        cl.addLayout(form)
        keys = QLabel(
            "Keyboard (works when ARMED, keys can be combined):\n"
            "  ← →  base     ↑ ↓  selected joint (1/2 = shoulder/elbow)\n"
            "  W S  shoulder     E D  elbow     G  toggle gripper\n"
            "  Space or Esc  EMERGENCY STOP\n"
            "EMG: fist ~0.6 s + relax = gripper, hold ~1.8 s = switch joint")
        keys.setStyleSheet("font-size: 12px; color: #94A3B8;")
        cl.addWidget(keys)
        cl.addStretch(1)
        row.addWidget(ctrl, 1)

        live = QGroupBox("Live")
        ll = QVBoxLayout(live)
        self.raw_label = QLabel("Model output: --")
        ll.addWidget(self.raw_label)
        self.action_label = QLabel("ACTION: HOLD")
        self.action_label.setStyleSheet("font-size: 20px; font-weight: 800; color: #38BDF8;")
        ll.addWidget(self.action_label)
        self.note_label = QLabel("")
        self.note_label.setWordWrap(True)
        self.note_label.setStyleSheet("color: #FBBF24;")
        ll.addWidget(self.note_label)
        self.joint_label = QLabel("Up/Down joint: " + self.vertical_combo.currentText().upper())
        self.joint_label.setStyleSheet("font-size: 15px; font-weight: 700;")
        ll.addWidget(self.joint_label)
        self.grip_hint_label = QLabel("")
        self.grip_hint_label.setStyleSheet("color: #38BDF8;")
        ll.addWidget(self.grip_hint_label)
        self.servo_bars = {}
        grid = QGridLayout()
        for r, (sid, servo) in enumerate(sorted(self.config["arm"]["servos"].items(), key=lambda kv: int(kv[0]))):
            grid.addWidget(QLabel(f"S{sid} {servo['name']}"), r, 0)
            bar = QProgressBar()
            bar.setRange(int(servo["min"]), int(servo["max"]))
            bar.setFormat("%v")
            bar.setValue(int(servo["min"]))
            grid.addWidget(bar, r, 1)
            self.servo_bars[int(sid)] = bar
        ll.addLayout(grid)
        self.gripper_label = QLabel("Gripper: unknown")
        ll.addWidget(self.gripper_label)
        self.key_label = QLabel("Keyboard: idle")
        self.key_label.setStyleSheet(themed_label_style("muted"))
        ll.addWidget(self.key_label)
        ll.addStretch(1)
        row.addWidget(live, 1)
        root.addLayout(row, 1)

    # ============================================================ armband
    def refresh_ports(self):
        self.port_combo.clear()
        self.port_combo.addItem("Simulator (local)", "socket://127.0.0.1:7000")
        for port in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{port.device} — {port.description}", port.device)

    def discover_wireless_devices(self):
        self.wireless_combo.clear()
        try:
            devices = ControlProtocol.discover()
        except Exception as exc:
            self.emg_status.setText(f"WIRELESS: discovery notice: {exc}")
            return []
        for device in devices:
            self.wireless_combo.addItem(device.summary, device)
        if devices:
            self.wireless_combo.setCurrentIndex(0)  # placeholder text otherwise leaves nothing selected
        self.emg_status.setText(f"WIRELESS: {len(devices)} ESP32 DEVICE(S) FOUND — press Connect Armband" if devices
                                else "WIRELESS: NO ESP32 FOUND — same Wi-Fi as this laptop? Or type its IP")
        self.emg_status.setStyleSheet(themed_label_style("success" if devices else "muted"))
        return devices

    def open_usb_provision_dialog(self):
        """Step 1 of the Wi-Fi flow (shared with mouse.py): probe the USB port, send Wi-Fi."""
        if self.emg_connected:
            QMessageBox.information(self, "USB Provision", "Disconnect the armband stream first.")
            return
        if USBProvisionDialog(self).exec_() == QDialog.Accepted:
            # The ESP32 restarts and joins Wi-Fi; keep looking for it for ~20 s.
            self.connection_mode.setCurrentIndex(self.connection_mode.findData("wireless"))
            self.discovery_attempts_left = 8
            self.emg_status.setText("WIRELESS: ESP32 restarting and joining Wi-Fi — searching…")
            self.emg_status.setStyleSheet(themed_label_style("muted"))
            QTimer.singleShot(4000, self.retry_discovery)

    def retry_discovery(self):
        if self.emg_connected:
            return
        self.discovery_attempts_left -= 1
        if self.discover_wireless_devices() or self.discovery_attempts_left <= 0:
            return
        self.emg_status.setText(
            f"WIRELESS: still searching for the ESP32… ({self.discovery_attempts_left} tries left)")
        QTimer.singleShot(2000, self.retry_discovery)

    def load_model(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Random Forest Model", str(TRAINED_MODEL_DIR),
                                              "Joblib Files (*.joblib)")
        if not path:
            return
        try:
            artifact = joblib.load(path)
            model = artifact["model"]
            classes = list(artifact["class_names"])
            input_channels = artifact.get("input_channels")
            if input_channels is None:
                input_channels = {70: 4, 156: 8, 231: 11}.get(getattr(model, "n_features_in_", 0), 4)
        except Exception as exc:
            QMessageBox.critical(self, "Model Error", f"Could not load the model:\n{exc}")
            return

        mapping = {c: canonical_gesture(c) for c in classes}
        unknown = [c for c, g in mapping.items() if g is None]
        missing = [g for g in ("left", "right", "up", "down", "grip", "rest") if g not in mapping.values()]
        if "switch" in mapping.values():
            # A dedicated switch gesture exists: the fist toggles the gripper at once.
            self.gesture_filter.switch_hold_s = 0.0
        else:
            self.gesture_filter.switch_hold_s = float(self.config["filter"].get("switch_hold_s", 0.0))
        if self.emg_connected:
            self.disconnect_emg()  # buffer sizes change with the model
        self.disarm("Model changed")
        self.window_samples = int(artifact.get("window_samples", 100))
        self.stride_samples = int(artifact.get("stride_samples", 25))
        self.num_channels = int(input_channels)
        self.data_buffer = np.zeros((self.num_channels, self.window_samples), dtype=np.float32)
        self.baseline = np.zeros(self.num_channels, dtype=np.float32)
        self.inference_worker.load_model(model, classes)
        self.gesture_filter.reset()
        self.model_loaded = True
        text = f"{Path(path).parent.name}  |  " + ", ".join(f"{c}→{mapping[c] or 'ignored'}" for c in classes)
        self.model_label.setText(text)
        self.model_label.setStyleSheet(themed_label_style("success"))
        if "rest" in missing:
            QMessageBox.warning(self, "No Rest class",
                                "This model has no Rest class, so it can never say 'do nothing'.\n"
                                "Use a model trained with a Rest gesture for arm control.")
        elif missing or unknown:
            QMessageBox.information(self, "Gesture mapping",
                                    f"Not in this model: {', '.join(missing) or 'none'}\n"
                                    f"Ignored classes: {', '.join(unknown) or 'none'}")
        self.update_controls()

    def toggle_emg(self):
        if self.emg_connected:
            self.disconnect_emg()
            return
        if not self.model_loaded:
            QMessageBox.warning(self, "Model Required", "Load an RF model before connecting the armband.")
            return
        try:
            if self.connection_mode.currentData() == "wireless":
                selected = self.wireless_combo.currentData()
                target_ip = selected.ip if isinstance(selected, DeviceInfo) else \
                    (self.wireless_combo.currentText().strip().split() or [""])[0]
                if not target_ip:
                    QMessageBox.warning(self, "Wireless Device", "Discover an ESP32 or enter its Wi-Fi IP address.")
                    return
                access_key = self.access_key_input.text().strip() or DEFAULT_ACCESS_KEY
                self.worker = WirelessStreamWorker(WIFI_STREAM_PORT)
                self.worker.batch_received.connect(self.on_batch)
                self.worker.error_occurred.connect(self.on_emg_error)
                self.worker.start()
                self.emg_status.setText(f"ARMBAND: connecting to {target_ip}…")
                QApplication.processEvents()
                try:
                    control_with_retry(ControlProtocol.start_stream, target_ip, access_key,
                                       get_local_ip_for_target(target_ip), str(WIFI_STREAM_PORT))
                except RuntimeError as exc:
                    raise RuntimeError(str(exc).splitlines()[0] + network_hint(target_ip)) from exc
                self.keepalive_worker = RetryingKeepaliveWorker(
                    target_ip, access_key, lambda: time.monotonic() - self.last_batch_t)
                self.keepalive_worker.notice.connect(self.on_keepalive_notice)
                self.keepalive_worker.start()
                self.current_wireless_device = (target_ip, access_key)
                text = f"ARMBAND: STREAMING FROM {target_ip}"
            else:
                port = self.port_combo.currentData()
                if not port:
                    return
                self.worker = SerialWorker(port, 921600, self.num_channels, batch_size=self.stride_samples)
                self.worker.batch_received.connect(self.on_batch)
                self.worker.error_occurred.connect(self.on_emg_error)
                self.worker.start()
                text = f"ARMBAND: STREAMING ({port})"
            self.emg_connected = True
            self.last_batch_t = time.monotonic()
            self.emg_button.setText("Disconnect Armband")
            self.emg_button.setStyleSheet(themed_button_style("danger"))
            self.emg_status.setText(text)
            self.emg_status.setStyleSheet(themed_label_style("success"))
        except Exception as exc:
            self.disconnect_emg()
            QMessageBox.critical(self, "Connection Error", str(exc))
        self.update_controls()

    def on_keepalive_notice(self, message):
        if not self.emg_connected:
            return
        if message == "restarted":
            self.emg_status.setText("ARMBAND: data had stopped — stream restarted")
            self.emg_status.setStyleSheet(themed_label_style("success"))
        else:
            # The arm already holds still on its own when EMG data stops.
            self.emg_status.setText(f"ARMBAND: no reply from ESP32 — check Wi-Fi ({message})")
            self.emg_status.setStyleSheet(themed_label_style("danger"))

    def on_emg_error(self, message):
        self.emergency_stop(f"Armband error: {message}")

    def disconnect_emg(self):
        if self.keepalive_worker:
            self.keepalive_worker.stop()
            self.keepalive_worker = None
        if self.current_wireless_device:
            try:
                control_with_retry(ControlProtocol.stop_stream, *self.current_wireless_device, attempts=2)
            except Exception:
                pass
            self.current_wireless_device = None
        if self.worker:
            self.worker.stop()
            self.worker = None
        self.emg_connected = False
        self.gesture_filter.reset()
        self.emg_button.setText("Connect Armband")
        self.emg_button.setStyleSheet(themed_button_style("success"))
        self.emg_status.setText("ARMBAND: OFFLINE")
        self.emg_status.setStyleSheet(themed_label_style("muted"))
        self.update_controls()

    def on_batch(self, batch):
        if self.data_buffer is None:
            return
        if isinstance(batch, dict):
            batch = batch.get("batch")
        if batch is None:
            return
        batch = np.asarray(batch, dtype=np.float32)
        if batch.ndim != 2 or not len(batch):
            return
        if batch.shape[1] < self.num_channels:
            batch = np.pad(batch, ((0, 0), (0, self.num_channels - batch.shape[1])))
        batch = batch[:, :self.num_channels]
        self.last_batch_t = time.monotonic()
        # Same adaptive baseline centering as the mouse controller and game.
        self.baseline *= 0.95
        self.baseline += 0.05 * np.mean(batch, axis=0, dtype=np.float32)
        incoming = (batch - self.baseline).T
        count = min(incoming.shape[1], self.window_samples)
        self.data_buffer[:, :-count] = self.data_buffer[:, count:]
        self.data_buffer[:, -count:] = incoming[:, -count:]
        self.sample_count += count
        if self.sample_count >= self.stride_samples:
            self.sample_count = 0
            self.inference_worker.submit_window(self.data_buffer.copy())

    def on_prediction(self, label, confidence):
        now = time.monotonic()
        gesture = self.gesture_filter.add_prediction(label, confidence, now)
        self.raw_label.setText(f"Model output: {label}  ({confidence * 100:.0f}%)  →  {gesture or 'ignored'}")
        self.log("prediction", label=label, confidence=f"{confidence:.3f}", gesture=gesture or "")

    # ================================================================ arm
    def toggle_arm(self):
        if self.arm:
            self.disconnect_arm()
            return
        simulate = self.simulate_check.isChecked()
        try:
            if simulate:
                start = dict(self.config["arm"]["home"])
                start[str(self.config["arm"]["gripper"]["servo"])] = self.config["arm"]["gripper"]["open"]
                backend = SimulatedBackend({int(k): int(v) for k, v in start.items()})
            else:
                backend = XArmBackend(self.config["arm"].get("connection", "USB"))
            arm = SafeArm(self.config, backend)
            streamer = MotionStreamer(arm, self.config["arm"])
        except ArmSafetyError as exc:
            QMessageBox.critical(self, "Config Error", f"config.json failed a safety check:\n{exc}")
            return
        except ImportError as exc:
            QMessageBox.critical(self, "Arm Library Missing",
                                 f"{exc}\n\nInstall it with:  pip install -r EMG_Robotic_Arm/requirements.txt")
            return
        except Exception as exc:
            QMessageBox.critical(self, "Arm Connection",
                                 f"Could not open the xArm over USB:\n{exc}\n\n"
                                 "Check the USB cable, that the arm's power switch is ON, and that no other "
                                 "program (e.g. the vegetable sorting script) is using the arm.")
            return

        positions = arm.read_positions()
        ok, voltage = arm.battery()
        arm.set_speed(self.speed_spin.value())
        self.arm = arm
        self.streamer = streamer
        streamer.start()
        self.teleop = Teleop(arm, self.gesture_filter, self.config, streamer)
        self.teleop.vertical = self.vertical_combo.currentData()
        self.open_log()
        unknown = [str(s) for s, p in positions.items() if p is None]
        volt = f"{voltage:.2f} V" if voltage else "voltage n/a"
        self.arm_status.setText(f"ARM: {backend.name} | {volt} | press Go Home")
        self.arm_status.setStyleSheet(themed_label_style("success" if ok else "danger"))
        self.arm_button.setText("Disconnect Arm")
        self.arm_button.setStyleSheet(themed_button_style("danger"))
        self.refresh_servo_view()
        self.log("arm_connected", note=f"{backend.name}; positions={json.dumps(positions)}; voltage={voltage}")
        if not ok:
            QMessageBox.warning(self, "Low Voltage",
                                f"Arm supply reads {voltage:.2f} V (minimum {arm.min_battery} V).\n"
                                "Use the 7.5 V adapter / charge the battery before moving the arm.")
        elif unknown:
            QMessageBox.information(self, "Positions",
                                    f"Could not read servo(s) {', '.join(unknown)}. Their first move will be extra slow.")
        self.update_controls()

    def disconnect_arm(self):
        self.disarm("Arm disconnected")
        if self.streamer:
            self.streamer.stop()
            self.streamer = None
        if self.arm:
            self.arm.close()  # torque stays on: the arm keeps holding its pose
        self.arm = None
        self.teleop = None
        self.arm_status.setText("ARM: OFFLINE")
        self.arm_status.setStyleSheet(themed_label_style("muted"))
        self.arm_button.setText("Connect Arm")
        self.arm_button.setStyleSheet(themed_button_style("success"))
        self.close_log()
        self.update_controls()

    def go_home(self):
        if not self.arm:
            return
        if QMessageBox.question(self, "Go Home",
                                "The arm will move slowly to its home pose.\n"
                                "Make sure nothing and nobody is within reach of the arm.\n\nContinue?"
                                ) != QMessageBox.Yes:
            return
        self.disarm("Homing")
        try:
            seconds = self.arm.go_home()
        except Exception as exc:
            self.handle_arm_failure(exc)
            return
        self.home_busy_until = time.monotonic() + seconds + 0.3
        self.arm_status.setText(f"ARM: homing ({seconds:.1f} s)…")
        self.log("home", note=f"{seconds:.1f}s")
        self.refresh_servo_view()
        QTimer.singleShot(int((seconds + 0.3) * 1000), self.after_home)
        self.update_controls()

    def after_home(self):
        if self.arm and self.arm.homed:
            self.arm_status.setText(f"ARM: {self.arm.backend.name} | HOME | ready to ARM")
        self.update_controls()

    def manual_gripper(self, closed):
        if not self.arm:
            return
        try:
            self.arm.set_gripper(closed)
            self.refresh_servo_view()
            self.log("gripper", note="closed" if closed else "open", source="button")
        except Exception as exc:
            self.handle_arm_failure(exc)

    def check_battery(self):
        if not self.arm or isinstance(self.arm.backend, SimulatedBackend):
            return
        if time.monotonic() - self.last_motion_t < 1.0 or (self.streamer and self.streamer.is_moving()):
            return  # a bus read would pause the motion stream
        ok, voltage = self.arm.battery()
        if voltage:
            self.arm_status.setText(self.arm_status.text().split(" | ")[0] + f" | {voltage:.2f} V")
        if not ok:
            self.emergency_stop(f"Arm voltage low ({voltage:.2f} V)")

    # =========================================================== arming
    def can_arm(self):
        return bool(self.arm and self.arm.homed and time.monotonic() >= self.home_busy_until)

    def toggle_armed(self):
        if self.teleop and self.teleop.armed:
            self.disarm("Disarmed by user")
            return
        if not self.can_arm():
            QMessageBox.information(self, "Not Ready", "Connect the arm and press 'Go Home' first.")
            return
        if self.emg_enable_check.isChecked() and not (self.emg_connected and self.model_loaded):
            if QMessageBox.question(self, "Armband not streaming",
                                    "The armband is not connected, so EMG gestures cannot be used.\n\n"
                                    "ARM in keyboard-only mode (arrow keys)?") != QMessageBox.Yes:
                return
            self.emg_enable_check.setChecked(False)
        self.gesture_filter.reset()  # never act on predictions made before arming
        self.pressed_keys.clear()
        self.streamer.clear_error()
        self.teleop.armed = True
        self.log("armed")
        self.update_controls()

    def not_ready_reason(self):
        if not self.arm:
            return "Arrow keys only move the arm when ARMED: first click 'Connect Arm'."
        if not self.arm.homed:
            return "Arrow keys only move the arm when ARMED: click 'Go Home' first, then ARM."
        if time.monotonic() < self.home_busy_until:
            return "Wait for homing to finish, then click ARM."
        return "Click the green ARM button to enable motion, then use the arrow keys."

    def disarm(self, reason=""):
        if self.streamer:
            try:
                self.streamer.halt()  # stop where it is, right now
            except Exception as exc:
                console(f"Halt failed: {exc}")
        if self.teleop and self.teleop.armed:
            self.teleop.armed = False
            self.log("disarmed", note=reason)
        self.pressed_keys.clear()
        self.key_grip_pending = False
        if hasattr(self, "key_label"):
            self.key_label.setText("Keyboard: idle")
        if reason:
            self.note_label.setText(reason)
        self.update_controls()

    def emergency_stop(self, reason):
        homing = time.monotonic() < self.home_busy_until
        parking = self.parking is not None
        self.parking = None
        if parking:
            self.park_aborted = True
            reason += ". Parking stopped, the arm holds here. Close again to park or exit"
        if self.arm and (homing or parking):
            # Homing/parking is one long servo command: stop it where the arm is now.
            try:
                self.arm.freeze()
            except Exception as exc:
                console(f"Freeze failed: {exc}")
            self.home_busy_until = 0.0
            self.refresh_servo_view()
        self.disarm(f"STOPPED: {reason}")
        self.action_label.setText("ACTION: STOPPED")

    def handle_arm_failure(self, exc):
        if isinstance(exc, ArmSafetyError):
            self.disarm(str(exc))
            QMessageBox.warning(self, "Safety", str(exc))
        else:
            self.emergency_stop(f"Arm communication error: {exc}")
            QMessageBox.critical(self, "Arm Error",
                                 f"{exc}\n\nMotion stopped. Check the USB cable and arm power, "
                                 "then Disconnect / Connect the arm again.")

    def on_vertical_changed(self):
        if self.teleop:
            self.teleop.vertical = self.vertical_combo.currentData()
        if hasattr(self, "joint_label"):
            self.joint_label.setText("Up/Down joint: " + self.vertical_combo.currentText().upper())

    def update_controls(self):
        armed = bool(self.teleop and self.teleop.armed)
        connected = self.arm is not None
        homing = time.monotonic() < self.home_busy_until or self.parking is not None
        self.simulate_check.setEnabled(not connected)
        self.home_button.setEnabled(connected and not armed and not homing)
        self.grip_open_button.setEnabled(connected and self.arm.homed and not homing)
        self.grip_close_button.setEnabled(connected and self.arm.homed and not homing)
        self.arm_toggle.setEnabled((armed or self.can_arm()) and self.parking is None)
        self.arm_toggle.setText("DISARM" if armed else "ARM (enable motion)")
        self.arm_toggle.setStyleSheet(themed_button_style("muted" if armed else "success"))
        simulated = connected and isinstance(self.arm.backend, SimulatedBackend)
        if self.parking:
            self.state_label.setText("PARKING ARM, THEN CLOSING  (Space/Esc = stop)")
        elif armed and simulated:
            self.state_label.setText("ARMED — SIMULATION ONLY (real arm will not move)")
        else:
            self.state_label.setText("ARMED — ARM WILL MOVE" if armed else "DISARMED")
        color = "#FBBF24" if self.parking else ("#F43F5E" if armed else "#94A3B8")
        self.state_label.setStyleSheet(f"font-size: 22px; font-weight: 800; color: {color};")

    # ========================================================== keyboard
    def handle_key(self, event, pressed):
        key = event.key()
        if key in STOP_KEYS:
            if pressed:
                self.emergency_stop("Space/Esc pressed")
            return True
        armed = bool(self.teleop and self.teleop.armed)
        if not armed:
            if key in KEY_MOTIONS or key == Qt.Key_G:
                # Consume so arrows never silently change the speed/joint boxes,
                # and say why the arm did not move.
                if pressed and not event.isAutoRepeat():
                    self.key_label.setText(f"Keyboard: {KEY_NAMES.get(key, 'G')} ignored (disarmed)")
                    self.note_label.setText(self.not_ready_reason())
                return True
            return False
        if event.isAutoRepeat():
            return key in KEY_MOTIONS
        if key in KEY_MOTIONS:
            if pressed:
                self.pressed_keys[key] = time.monotonic()
            else:
                self.pressed_keys.pop(key, None)
            held = [KEY_NAMES[k] for k in self.pressed_keys]
            self.key_label.setText(f"Keyboard: {' '.join(held)} held" if held else "Keyboard: idle")
            return True
        if pressed and key == Qt.Key_G:
            self.key_grip_pending = True
            return True
        if pressed and key in (Qt.Key_1, Qt.Key_2):
            index = 0 if key == Qt.Key_1 else 1
            if index < self.vertical_combo.count():
                self.vertical_combo.setCurrentIndex(index)
            return True
        return False

    # ====================================================== control loop
    def control_tick(self):
        now = time.monotonic()
        if not (self.teleop and self.teleop.armed):
            return
        if self.streamer.error is not None:
            exc, self.streamer.error = self.streamer.error, None
            self.handle_arm_failure(exc)
            return

        key_motions = {KEY_MOTIONS[k]: now - since for k, since in self.pressed_keys.items()}
        key_grip, self.key_grip_pending = self.key_grip_pending, False

        emg_enabled = self.emg_enable_check.isChecked()
        emg_ok = (emg_enabled and self.emg_connected
                  and now - self.last_batch_t <= float(self.config["filter"]["emg_timeout_s"]))
        try:
            status = self.teleop.tick(now, key_motions, key_grip, emg_ok)
        except Exception as exc:
            self.handle_arm_failure(exc)
            return

        note = status["note"]
        if not emg_enabled and (not note or note.startswith("EMG stream lost")):
            note = "Keyboard only"
        self.note_label.setText(note)
        self.grip_hint_label.setText(self.gesture_filter.grip_hint(now) if emg_ok else "")
        if status["motion"]:
            self.action_label.setText(f"ACTION: {status['motion'].upper()}  ({status['source']})")
        else:
            self.action_label.setText("ACTION: HOLD")
        if status["switched"]:
            index = self.vertical_combo.findData(status["switched"])
            self.vertical_combo.setCurrentIndex(index)  # also updates joint_label
            self.log("switch_joint", note=status["switched"], source=status["source"])
        if status["gripper"]:
            self.log("gripper", note=status["gripper"], source=status["source"])
        if status["motion"] or status["gripper"]:
            self.last_motion_t = now
            self.log("move", motion=status["motion"] or "", source=status["source"] or "", note=note)
        if status["motion"] or status["gripper"] or self.streamer.is_moving():
            self.refresh_servo_view()

    def refresh_servo_view(self):
        if not self.arm:
            return
        targets = dict(self.arm.targets)  # updated by the motion thread
        for sid, bar in self.servo_bars.items():
            pos = targets.get(sid)
            if pos is not None:
                bar.setValue(max(bar.minimum(), min(bar.maximum(), pos)))
        if self.arm.gripper_closed is not None:
            self.gripper_label.setText(f"Gripper: {'CLOSED' if self.arm.gripper_closed else 'OPEN'}")

    # =========================================================== logging
    def open_log(self):
        LOG_DIR.mkdir(exist_ok=True)
        path = LOG_DIR / f"session_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = open(path, "w", newline="", encoding="utf-8")
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=[
            "time_s", "event", "label", "confidence", "gesture", "motion", "source", "note", "targets"])
        self.log_writer.writeheader()
        self.log_t0 = time.monotonic()

    def log(self, event, **fields):
        if not self.log_writer:
            return
        row = {"time_s": f"{time.monotonic() - self.log_t0:.3f}", "event": event,
               "targets": json.dumps(self.arm.targets) if self.arm else ""}
        row.update(fields)
        self.log_writer.writerow(row)

    def close_log(self):
        if self.log_file:
            self.log_file.close()
        self.log_file = None
        self.log_writer = None

    # ============================================================= close
    def closeEvent(self, event):
        if self.close_ready:
            self.shutdown()
            event.accept()
            return
        event.ignore()  # park first; finish_close() closes the window afterwards
        if self.parking:
            self.note_label.setText("Parking the arm before exit… (Space/Esc stops it)")
            return
        if not (self.arm and self.config["arm"].get("park_on_exit", True)):
            self.finish_close()
            return
        if self.park_aborted:
            answer = QMessageBox.question(
                self, "Exit",
                "Parking was stopped.\n\nYes: park the arm at home now, then exit.\n"
                "No: exit and leave the arm holding where it is.",
                QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
            if answer == QMessageBox.Cancel:
                return
            if answer == QMessageBox.No:
                self.finish_close()
                return
        self.start_parking()

    # ------------------------------------------------- park before exit
    def start_parking(self):
        self.parking = "home"
        self.park_aborted = False
        self.disarm("Closing: parking the arm at home first (Space/Esc stops it)")
        try:
            seconds = self.arm.go_home()
        except Exception as exc:
            self.log("park", note=f"failed: {exc}")
            console(f"Parking failed: {exc}")
            self.parking = None
            QMessageBox.critical(self, "Parking failed",
                                 f"Could not move the arm home before exiting:\n{exc}\n\n"
                                 "The app will close and the arm keeps holding its pose.")
            self.finish_close()
            return
        real = not isinstance(self.arm.backend, SimulatedBackend)
        wait = seconds + 0.3 if real else 0.2
        self.home_busy_until = time.monotonic() + wait
        self.arm_status.setText(f"ARM: parking at home ({seconds:.1f} s)…")
        self.log("park", note=f"home {seconds:.1f}s")
        self.refresh_servo_view()
        self.update_controls()
        QTimer.singleShot(int(wait * 1000), self.park_open_gripper)

    def park_open_gripper(self):
        if self.parking != "home":
            return  # stopped by Space/Esc
        self.home_busy_until = 0.0
        if not (self.config["arm"].get("open_gripper_on_exit", True) and self.arm.gripper_closed is not False):
            self.park_done()
            return
        self.parking = "gripper"
        try:
            self.arm.set_gripper(False)
        except Exception as exc:
            console(f"Opening the gripper failed: {exc}")
            self.park_done()
            return
        self.log("park", note="gripper opened")
        self.refresh_servo_view()
        real = not isinstance(self.arm.backend, SimulatedBackend)
        # The speed cap can stretch the gripper move beyond grip_ms; wait for all of it.
        travel_ms = abs(self.arm.grip_close - self.arm.grip_open) / self.arm.hard_max_speed * 1000
        QTimer.singleShot(int(max(self.arm.grip_ms, travel_ms)) + 200 if real else 100, self.park_done)

    def park_done(self):
        if self.parking is None:
            return  # stopped by Space/Esc
        self.parking = None
        errors = {}
        if not isinstance(self.arm.backend, SimulatedBackend):
            try:
                errors = self.arm.home_errors()
            except Exception as exc:
                console(f"Could not check the home pose: {exc}")
        if errors:
            detail = "\n".join(f"  {self.arm.names[sid]} (servo {sid}): at {pos if pos is not None else '?'}, "
                                f"home is {home}" for sid, (pos, home) in sorted(errors.items()))
            self.log("park", note=f"not at home: {errors}")
            console("Arm did not fully reach home:\n" + detail)
            QMessageBox.warning(self, "Arm not fully parked",
                                "The arm did not fully reach its home pose:\n" + detail +
                                "\n\nSomething may be blocking it. The app will now close.")
        else:
            self.log("park", note="at home")
        self.finish_close()

    def finish_close(self):
        self.close_ready = True
        self.close()

    def shutdown(self):
        self.control_timer.stop()
        self.battery_timer.stop()
        if self.streamer:
            self.streamer.stop()
        if self.emg_connected:
            self.disconnect_emg()
        if self.arm:
            self.disconnect_arm()  # torque stays on: the arm keeps holding its pose
        self.inference_worker.stop()


def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else None
    config = load_config(config_path)
    app = QApplication(sys.argv)
    apply_dark_theme(app, font_size=14)
    window = EmgArmWindow(config)

    # Ctrl+C, `kill` or closing the terminal go through closeEvent too, so the
    # arm is parked first. (Python only runs signal handlers while it executes
    # Python code, hence the idle timer.)
    def request_close(*_):
        QTimer.singleShot(0, window.close)
    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), request_close)
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(200)

    # An unexpected error must not kill the app (PyQt would abort, skipping the
    # parking): stop motion, report it, and keep the window open.
    def on_error(etype, value, tb):
        console("".join(traceback.format_exception(etype, value, tb)))
        try:
            window.log("error", note=f"{etype.__name__}: {value}")
            window.emergency_stop(f"Internal error ({etype.__name__}: {value})")
        except Exception:
            pass
    sys.excepthook = on_error

    key_filter = KeyboardFilter(window)
    app.installEventFilter(key_filter)
    app.applicationStateChanged.connect(
        lambda state: state != Qt.ApplicationActive and window.pressed_keys.clear())
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
