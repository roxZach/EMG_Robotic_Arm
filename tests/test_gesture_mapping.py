"""Choosing what each model class does, no hardware:
python -m unittest discover tests"""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from PyQt5.QtWidgets import QApplication  # noqa: E402

from emg_arm.app.gesture_mapping import IGNORE, GestureMappingDialog, class_training_stats  # noqa: E402
from emg_arm.arm.driver import load_config  # noqa: E402
from emg_arm.arm.gesture_control import GestureFilter, load_gesture_mapping, save_gesture_mapping  # noqa: E402

CONFIG = load_config()
CLASSES = ["Left", "Rest", "Right", "down", "fist_close", "up"]
DEFAULTS = {"Left": "left", "Rest": "rest", "Right": "right", "down": "down",
            "fist_close": "grip", "up": "up"}
APP = QApplication.instance() or QApplication([])


def feed(gesture_filter, label, t=0.0, count=8):
    for _ in range(count):
        gesture_filter.add_prediction(label, 0.95, t)
        t += 0.05
    return t


class FilterMappingTests(unittest.TestCase):
    def test_custom_mapping_changes_the_motion(self):
        f = GestureFilter(CONFIG["filter"])
        f.label_map = {"up": "left"}
        t = feed(f, "up")
        self.assertEqual(f.motion(t), "left")

    def test_ignored_class_never_moves(self):
        f = GestureFilter(CONFIG["filter"])
        f.label_map = {"up": None}
        t = feed(f, "up")
        self.assertIsNone(f.motion(t))

    def test_unmapped_class_uses_its_name(self):
        f = GestureFilter(CONFIG["filter"])
        f.label_map = {"up": "left"}
        t = feed(f, "down")
        self.assertEqual(f.motion(t), "down")

    def test_mapping_survives_filter_reset(self):
        f = GestureFilter(CONFIG["filter"])
        f.label_map = {"up": "left"}
        f.reset()
        self.assertEqual(f.action_for("up"), "left")


class SavedMappingTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.path = Path(tmp.name) / "gesture_mapping.json"

    def test_missing_file_uses_names(self):
        self.assertEqual(load_gesture_mapping(self.path, CLASSES), DEFAULTS)

    def test_round_trip_keeps_ignored_classes(self):
        mapping = dict(DEFAULTS, up="grip", fist_close=None)
        save_gesture_mapping(self.path, mapping)
        self.assertEqual(load_gesture_mapping(self.path, CLASSES), mapping)

    def test_bad_entries_fall_back_to_names(self):
        self.path.write_text(json.dumps({"up": "fly", "down": ["left"], "gone": "left"}))
        self.assertEqual(load_gesture_mapping(self.path, CLASSES), DEFAULTS)

    def test_corrupt_file_falls_back_to_names(self):
        self.path.write_text("not json")
        self.assertEqual(load_gesture_mapping(self.path, CLASSES), DEFAULTS)
        self.path.write_text("[1, 2]")
        self.assertEqual(load_gesture_mapping(self.path, CLASSES), DEFAULTS)


class DialogTests(unittest.TestCase):
    def dialog(self, mapping=DEFAULTS):
        return GestureMappingDialog(CLASSES, mapping, {}, None)

    def choose(self, dialog, name, action):
        combo = dialog.combos[name]
        combo.setCurrentIndex(combo.findData(action))

    def test_shows_current_mapping(self):
        mapping = dict(DEFAULTS, up="grip", down=None)
        self.assertEqual(self.dialog(mapping).mapping(), mapping)

    def test_needs_a_rest_gesture_to_save(self):
        dialog = self.dialog()
        self.assertTrue(dialog.save_button.isEnabled())
        self.choose(dialog, "Rest", IGNORE)
        self.assertFalse(dialog.save_button.isEnabled())

    def test_warns_when_relaxed_hand_would_move_the_arm(self):
        dialog = self.dialog()
        self.choose(dialog, "up", "rest")
        self.choose(dialog, "Rest", "left")
        self.assertIn("relaxed hand", dialog.note.text())

    def test_names_unassigned_actions(self):
        dialog = self.dialog()
        self.choose(dialog, "down", IGNORE)
        self.assertIn("Down", dialog.note.text())

    def test_reset_restores_defaults(self):
        dialog = self.dialog(dict(DEFAULTS, up="grip"))
        dialog.reset_defaults()
        self.assertEqual(dialog.mapping(), DEFAULTS)


class TrainingStatsTests(unittest.TestCase):
    def test_reads_windows_and_accuracy(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "training_results.json"
            path.write_text(json.dumps({"classes": ["Left", "Rest"],
                                        "class_window_counts": {"Left": 40, "Rest": 90},
                                        "confusion_matrix": [[3, 1], [0, 5]]}))
            self.assertEqual(class_training_stats(path, ["Left", "Rest"]),
                             {"Left": (40, 0.75), "Rest": (90, 1.0)})

    def test_missing_results_are_not_an_error(self):
        self.assertEqual(class_training_stats("/nonexistent/training_results.json", CLASSES), {})


if __name__ == "__main__":
    unittest.main()
