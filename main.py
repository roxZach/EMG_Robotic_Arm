"""EMG Robotic Arm Control: start here.

    python3 main.py                  start the app
    python3 main.py my_config.json   start it with another config file

The first run sets up the project's Python environment (.venv) by itself.

How the code is organised
-------------------------
    emg_arm/armband/   talk to the BioWave armband and recognise gestures
        link.py              Wi-Fi/USB connection, USB provisioning, model inference thread
        features.py          EMG features fed to the Random Forest model
    emg_arm/arm/       move the xArm 1S safely
        gesture_control.py   gestures and keys -> joint velocities (vote filter, lockout, mapping)
        motion.py            smooth motion, sent to the arm from its own thread
        driver.py            joint limits, speed cap, homing, gripper (real and simulated arm)
    emg_arm/app/       the windows you see
        main_window.py       the main window
        gesture_mapping.py   the Gesture Mapping window
        theme.py             colours and styles
    emg_arm/env_bootstrap.py creates and uses .venv automatically
    tools/             separate helper programs (arm hardware check, armband simulator)
    config.json        arm limits, home pose, which joint each gesture moves, filter settings

Data flow:
    armband -> link.py -> features.py + model -> gesture_control.py -> motion.py -> driver.py -> arm
"""

import signal
import sys
import traceback

from emg_arm import env_bootstrap

env_bootstrap.ensure()  # restarts inside the project's .venv, creating it on the first run

from PyQt5.QtCore import Qt, QTimer  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from emg_arm.app.main_window import EmgArmWindow, KeyboardFilter, console  # noqa: E402
from emg_arm.app.theme import apply_dark_theme  # noqa: E402
from emg_arm.arm.driver import load_config  # noqa: E402


def main():
    """Start the app. An optional argument is another config file."""
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
