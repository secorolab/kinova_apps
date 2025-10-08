import os
import datetime
from typing import List, Optional

import cv2

from PySide6.QtCore import Qt, QTimer, QSize
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QLineEdit, QPushButton, QMessageBox
)

from kinova_apps.gui.experiment_manager import ExperimentManager


class CameraPanel(QWidget):
    def __init__(self, exp: ExperimentManager, parent=None):
        super().__init__(parent)
        self.exp = exp

        self.url_input = QLineEdit("http://192.168.0.101:8080/video")
        self.start_btn = QPushButton("Start Feed")
        self.stop_btn = QPushButton("Stop Feed")
        self.stop_btn.setEnabled(False)

        self.rec_btn = QPushButton("Start Video Rec")
        self.rec_stop_btn = QPushButton("Stop Video Rec")
        self.rec_stop_btn.setEnabled(False)

        self.view = QLabel("Camera feed")
        self.view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.view.setMinimumSize(QSize(480, 320))
        self.view.setScaledContents(True)

        h = QHBoxLayout()
        h.addWidget(self.url_input, 1)
        h.addWidget(self.start_btn)
        h.addWidget(self.stop_btn)

        h2 = QHBoxLayout()
        h2.addWidget(self.rec_btn)
        h2.addWidget(self.rec_stop_btn)

        box = QGroupBox("Camera")
        inner = QVBoxLayout()
        inner.addLayout(h)
        inner.addWidget(self.view)
        inner.addLayout(h2)
        box.setLayout(inner)

        lay = QVBoxLayout(self)
        lay.addWidget(box)

        self.cap = None
        self.timer = QTimer(self)
        self.timer.setInterval(30)
        self.timer.timeout.connect(self._update_frame)
        self.start_btn.clicked.connect(self.start)
        self.stop_btn.clicked.connect(self.stop)

        self.rec_btn.clicked.connect(self.start_recording)
        self.rec_stop_btn.clicked.connect(self.stop_recording)

        # Video recording state
        self.vwriter = None
        self.rec_start_monotonic: Optional[float] = None
        self.last_frame_size: Optional[tuple[int, int]] = None
        self.fps_estimate = 30.0  # fallback

        # Event markers (relative to rec start, seconds)
        self.event_markers: List[dict] = []

    def refresh(self):
        self.setup_logs()

    def setup_logs(self):
        run_dir = self.exp.ensure_run()
        self.videos_dir = os.path.join(run_dir, "videos")
        os.makedirs(self.videos_dir, exist_ok=True)

    def start(self):
        url = self.url_input.text().strip()
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            self.view.setText("Failed to open stream.")
            if self.cap:
                self.cap.release()
                self.cap = None
            return
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.timer.start()

    def stop(self):
        self.timer.stop()
        if self.cap:
            self.cap.release()
            self.cap = None
        self.view.setText("Camera feed")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.stop_recording()

    def _ensure_writer(self, frame) -> bool:
        if self.vwriter is not None:
            return True
        h, w, _ = frame.shape
        self.last_frame_size = (w, h)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.exp.ensure_run()
        out_path = os.path.join(self.videos_dir, f"{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4")
        self.vwriter = cv2.VideoWriter(out_path, fourcc, self.fps_estimate, (w, h))
        if not self.vwriter.isOpened():
            self.vwriter = None
            QMessageBox.critical(self, "Camera", "Failed to open video writer.")
            return False
        self.rec_start_monotonic = datetime.datetime.now().timestamp()
        self.event_markers = []
        return True

    def start_recording(self):
        if self.exp.ensure_run() is None:
            QMessageBox.warning(self, "Camera", "Please create or select an experiment first.")
            return
        if self.cap is None or not self.cap.isOpened():
            QMessageBox.warning(self, "Camera", "Start the camera feed first.")
            return
        # Will actually open writer on first frame in _update_frame
        self.rec_btn.setEnabled(False)
        self.rec_stop_btn.setEnabled(True)

    def stop_recording(self):
        if self.vwriter is not None:
            try:
                self.vwriter.release()
            except Exception:
                pass
            self.vwriter = None
        self.rec_btn.setEnabled(True)
        self.rec_stop_btn.setEnabled(False)
        # Persist markers if any
        if self.event_markers:
            self.exp.write_json(self.videos_dir, "video_event_markers.json", {"events": self.event_markers})

    def mark_event(self, label: str):
        if self.rec_start_monotonic is None:
            return
        rel_t = datetime.datetime.now().timestamp() - self.rec_start_monotonic
        self.event_markers.append({"label": label, "t_rel_sec": rel_t})

    def _update_frame(self):
        if not self.cap:
            return
        ok, frame = self.cap.read()
        if not ok:
            return

        # On first successful frame, probe FPS
        if self.last_frame_size is None:
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            if fps and fps > 0:
                self.fps_estimate = float(fps)

        # Write video if enabled
        if self.rec_stop_btn.isEnabled():
            if self._ensure_writer(frame):
                self.vwriter.write(frame)

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame_rgb.shape
        qimg = QImage(frame_rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
        self.view.setPixmap(QPixmap.fromImage(qimg))

    def close(self):
        self.stop()
        return super().close()
