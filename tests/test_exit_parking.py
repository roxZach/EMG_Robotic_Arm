"""Stopping a long move (homing/parking) and checking the parked pose, no hardware:
python -m unittest discover tests"""

import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from emg_arm.arm.driver import SafeArm, SimulatedBackend, load_config  # noqa: E402

CONFIG = load_config()


class HalfwayBackend(SimulatedBackend):
    """Reports every servo halfway to its last target, like a real servo mid-move."""

    def __init__(self, start):
        super().__init__(start)
        self.start = dict(start)

    def get_position(self, servo_id):
        target = self.positions.get(servo_id)
        if target is None:
            return None
        return (self.start.get(servo_id, target) + target) // 2


class FreezeTests(unittest.TestCase):
    def setUp(self):
        self.backend = HalfwayBackend({1: 500, 2: 500, 4: 600, 5: 400, 6: 200})
        self.arm = SafeArm(CONFIG, self.backend)
        self.arm.read_positions()
        self.backend.start = {sid: self.arm.targets[sid] for sid in self.arm.targets}

    def test_freeze_stops_a_homing_move_where_the_arm_is(self):
        self.arm.go_home()
        self.backend.packets.clear()
        positions = self.arm.freeze()
        self.assertEqual(len(self.backend.packets), 1, "one packet for all joints")
        packet, _ms = self.backend.packets[0]
        for sid in (4, 5, 6):
            self.assertEqual(packet[sid], positions[sid])
            self.assertEqual(self.arm.targets[sid], positions[sid])
        self.assertEqual(packet[6], (200 + 500) // 2, "base stops halfway, not at home")

    def test_freeze_leaves_the_gripper_alone(self):
        self.arm.go_home()
        self.backend.packets.clear()
        self.arm.freeze()
        self.assertNotIn(self.arm.grip_servo, self.backend.packets[0][0],
                         "re-commanding a gripping servo would loosen the grip")

    def test_home_errors(self):
        arm = SafeArm(CONFIG, SimulatedBackend({1: 100, 2: 500, 4: 820, 5: 700, 6: 500}))
        arm.read_positions()
        self.assertEqual(arm.home_errors(), {})
        arm.backend.positions[6] = 300  # base stuck away from home
        self.assertEqual(arm.home_errors(), {6: (300, 500)})


class WindowCloseTests(unittest.TestCase):
    def test_closes_with_no_arm_connected(self):
        from PyQt5.QtCore import QTimer
        from PyQt5.QtWidgets import QApplication

        from emg_arm.app.main_window import EmgArmWindow

        app = QApplication.instance() or QApplication([])
        window = EmgArmWindow(CONFIG)
        self.addCleanup(window.inference_worker.stop)
        window.show()
        QTimer.singleShot(0, window.close)
        QTimer.singleShot(2000, app.quit)  # safety net if the window never closes
        app.exec_()
        self.assertFalse(window.isVisible())


if __name__ == "__main__":
    unittest.main()
