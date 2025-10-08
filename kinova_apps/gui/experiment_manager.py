import os
import json
import datetime
from typing import Optional

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QPushButton, QFileDialog
)
from PySide6.QtCore import Signal


class ExperimentManager:
    def __init__(self, root: str = "experiment_logs") -> None:
        self.root = os.path.abspath(root)
        # os.makedirs(self.root, exist_ok=True)
        self.current_run_dir: Optional[str] = None

    def new_run(self) -> str:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(self.root, ts)
        self.current_run_dir = run_dir
        return run_dir

    def ensure_run(self) -> Optional[str]:
        if not self.current_run_dir:
            return None
        return self.current_run_dir

    def write_json(self, path: str, name: str, data: dict) -> str:
        path = os.path.join(path, name)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return path


class ExpertimentsPanel(QWidget):
    new_exp_signal = Signal(name="new_experiment")

    def __init__(self, exp_manager: ExperimentManager, parent=None) -> None:
        super().__init__(parent)
        self.exp = exp_manager
        
        # Experiment controls
        self.exp_dir_label = QLabel("No experiment folder selected... Select or create one.")
        self.btn_new_exp = QPushButton("New Experiment")
        self.btn_new_exp.clicked.connect(self.refresh)

        self.btn_pick_expdir = QPushButton("Select Experiments Dir")
        self.btn_pick_expdir.clicked.connect(self._pick_experiments_dir)

        exp_box = QGroupBox("Experiment")
        exp_lay = QHBoxLayout()
        exp_lay.addWidget(self.exp_dir_label, 1)
        exp_lay.addWidget(self.btn_pick_expdir)
        exp_lay.addWidget(self.btn_new_exp)
        exp_box.setLayout(exp_lay)

        lay = QVBoxLayout(self)
        lay.addWidget(exp_box)
        self.setLayout(lay)

    def refresh(self):
        self.exp.new_run()
        self.exp_dir_label.setText(f"Current experiment dir: {self.exp.current_run_dir}")
        self.new_exp_signal.emit()

    def reset(self):
        self.exp.current_run_dir = None
        self.exp_dir_label.setText("No experiment folder selected... Select or create one.")

    def _pick_experiments_dir(self):
        """Let user pick a new root directory for experiments."""
        d = QFileDialog.getExistingDirectory(self, "Select Experiments Directory", os.path.abspath("."))
        if not d:
            return
        self.exp.root = d
        os.makedirs(self.exp.root, exist_ok=True)
        self.exp_dir_label.setText(f"Experiments root: {d}")

