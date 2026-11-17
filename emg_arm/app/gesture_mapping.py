"""Dialog for choosing what each gesture in the loaded model does on the arm.

The rows come from the model's own classes, i.e. the gestures in the data it was
trained on. training_results.json (saved next to the model) adds how much data
each gesture has and how well the model recognised it, when it is available.
"""

import json
from pathlib import Path

from PyQt5.QtWidgets import (
    QComboBox, QDialog, QGridLayout, QHBoxLayout, QLabel, QPushButton, QVBoxLayout,
)

from emg_arm.app.theme import themed_button_style, themed_label_style
from emg_arm.arm.gesture_control import MOTIONS, canonical_gesture

IGNORE = "ignore"  # combo box value for a class mapped to None
ACTION_CHOICES = (
    ("Left", "left"), ("Right", "right"), ("Up", "up"), ("Down", "down"),
    ("Grip (toggle gripper)", "grip"), ("Switch Up/Down joint", "switch"),
    ("Rest (hold still)", "rest"), ("Ignore", IGNORE),
)
WEAK_ACCURACY = 0.7  # shown in red: a poor choice for an important action


def class_training_stats(results_path, classes):
    """{class: (training windows, test accuracy)} from training_results.json.
    Returns {} if the file is missing or unreadable; the stats are only a guide."""
    try:
        results = json.loads(Path(results_path).read_text(encoding="utf-8"))
        counts = results["class_window_counts"]
        stats = {}
        # Confusion matrix rows are the true class, so the diagonal share is recall.
        for i, (name, row) in enumerate(zip(results["classes"], results["confusion_matrix"])):
            if name in classes and sum(row):
                stats[name] = (counts[name], row[i] / sum(row))
        return stats
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        return {}


class GestureMappingDialog(QDialog):
    def __init__(self, classes, mapping, stats, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Gesture Mapping")
        self.setModal(True)
        self.classes = list(classes)
        self.combos = {}

        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        intro = QLabel("Choose what each gesture in this model does. "
                       "The list comes from the gestures the model was trained on.")
        intro.setWordWrap(True)
        layout.addWidget(intro)

        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        for col, title in enumerate(("Gesture", "Windows", "Test accuracy", "Arm action")):
            header = QLabel(title)
            header.setStyleSheet(themed_label_style("muted"))
            grid.addWidget(header, 0, col)
        for row, name in enumerate(self.classes, start=1):
            grid.addWidget(QLabel(name), row, 0)
            windows, accuracy = stats.get(name, (None, None))
            grid.addWidget(QLabel("—" if windows is None else str(windows)), row, 1)
            accuracy_label = QLabel("—" if accuracy is None else f"{accuracy * 100:.0f}%")
            if accuracy is not None and accuracy < WEAK_ACCURACY:
                accuracy_label.setStyleSheet(themed_label_style("danger"))
            grid.addWidget(accuracy_label, row, 2)
            combo = QComboBox()
            combo.setMinimumSize(210, 32)
            for label, action in ACTION_CHOICES:
                combo.addItem(label, action)
            self.select(combo, mapping.get(name))
            combo.currentIndexChanged.connect(self.refresh_note)
            grid.addWidget(combo, row, 3)
            self.combos[name] = combo
        layout.addLayout(grid)

        hint = QLabel("Without a Switch gesture, holding the Grip gesture longer switches the "
                      "Up/Down joint instead. Left, Right, Up and Down still move the joints "
                      "set in config.json.")
        hint.setWordWrap(True)
        hint.setStyleSheet(themed_label_style("muted"))
        layout.addWidget(hint)

        self.note = QLabel()
        self.note.setWordWrap(True)
        layout.addWidget(self.note)

        buttons = QHBoxLayout()
        reset = QPushButton("Reset to Defaults")
        reset.setStyleSheet(themed_button_style("muted"))
        reset.clicked.connect(self.reset_defaults)
        buttons.addWidget(reset)
        buttons.addStretch(1)
        cancel = QPushButton("Cancel")
        cancel.setStyleSheet(themed_button_style("muted"))
        cancel.clicked.connect(self.reject)
        buttons.addWidget(cancel)
        self.save_button = QPushButton("Save Mapping")
        self.save_button.setStyleSheet(themed_button_style("success"))
        self.save_button.clicked.connect(self.accept)
        buttons.addWidget(self.save_button)
        layout.addLayout(buttons)

        self.refresh_note()

    @staticmethod
    def select(combo, action):
        combo.setCurrentIndex(combo.findData(action or IGNORE))

    def mapping(self):
        """{class: action}, with None for an ignored class."""
        result = {}
        for name, combo in self.combos.items():
            action = combo.currentData()
            result[name] = None if action == IGNORE else action
        return result

    def reset_defaults(self):
        for name, combo in self.combos.items():
            self.select(combo, canonical_gesture(name))

    def refresh_note(self):
        mapping = self.mapping()
        chosen = set(mapping.values())
        has_rest = "rest" in chosen
        relaxed_moves = [name for name in self.classes
                         if canonical_gesture(name) == "rest" and mapping[name] in MOTIONS + ("grip", "switch")]
        missing = [label.split(" (")[0] for label, action in ACTION_CHOICES
                   if action in MOTIONS + ("grip",) and action not in chosen]
        self.save_button.setEnabled(has_rest)
        if not has_rest:
            text, kind = "Set at least one gesture to Rest, so the arm has a way to hold still.", "danger"
        elif relaxed_moves:
            text, kind = f"Warning: {', '.join(relaxed_moves)} is your relaxed hand. It would now act on the arm.", "danger"
        elif missing:
            text, kind = f"Not assigned to any gesture: {', '.join(missing)}.", "muted"
        else:
            text, kind = "All actions are assigned.", "success"
        self.note.setText(text)
        self.note.setStyleSheet(themed_label_style(kind))
