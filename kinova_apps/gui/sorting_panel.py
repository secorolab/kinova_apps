import datetime
from typing import List, Literal
import threading
import json

from PySide6 import QtGui
from PySide6.QtCore import Qt, Signal, SignalInstance, QEnum
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QFormLayout, QPushButton, QCheckBox, QTextEdit, QMessageBox, QScrollArea,
    QTableWidget, QTableWidgetItem
)

from pydantic import BaseModel
from enum import StrEnum

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import String

from kinova_apps.gui.camera_panel import CameraPanel
from kinova_apps.gui.experiment_manager import ExperimentManager

# ---------------------------- Data Models ----------------------------

class PickPlaceResponse(BaseModel):
    status: Literal["successful", "failure"]
    pickplace_failed: bool
    unsafe: bool
    reasoning: str


class SortingResponse(BaseModel):
    status: Literal["successful", "failure"]
    block_status: List[PickPlaceResponse]
    wrong_sorting: bool
    reasoning: str


class TaskStatus(StrEnum):
    NOT_STARTED = "not_started"
    WAITING = "waiting"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    NONE = "none"


class TaskControl(StrEnum):
    START = "start"
    STOP = "stop"
    WAIT = "wait"
    START_SORTING = "start_sorting"
    CONTINUE = "continue"
    REDO_DETECT = "redo_detect"


class RosNode(Node):
    def __init__(self, *, 
                 context=None,
                 status_signal: SignalInstance,
                 selections_signal: SignalInstance
    ):
        super().__init__("sorting_task_node", context=context)

        self.task_control_topic = "sorting_task/control"
        self.task_status_topic  = "sorting_task/status"
        self.selections_topic   = "sorting_task/selections"
         
        self.pub_task_control = self.create_publisher(String, self.task_control_topic, 10)
 
        self.status_signal = status_signal
        self.sub_task_status = self.create_subscription(
            String,
            self.task_status_topic,
            self._callback_task_status,
            10
        )

        self.selections = None
        self.selections_signal = selections_signal
        self.sub_selections = self.create_subscription(
            String,
            self.selections_topic,
            self._callback_selections,
            1
        )

        self.sorting_task_status: TaskStatus = TaskStatus.NONE

    def _callback_task_status(self, msg: String):
        self.sorting_task_status = TaskStatus(msg.data)
        self.status_signal.emit()

    def _callback_selections(self, msg: String):
        json_string = msg.data
        try:
            self.selections = json.loads(json_string)
            self.selections_signal.emit()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode selections JSON: {e}")


class RosThread(threading.Thread):
    def __init__(self, *, context, node: RosNode):
        super().__init__(daemon=True)
        self.context = context
        self._stop_event = threading.Event()
        self._executor = None
        self._node = node

    def run(self):
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self._node)
        
        try:
            while not self._stop_event.is_set():
                self._executor.spin_once(timeout_sec=0.1)
        finally:
                self._executor.remove_node(self._node)
                self._node.destroy_node()

    def stop(self):
        self._stop_event.set()


COLOR_MAP = {
    "yellow": "#FFD700",
    "blue":   "#4DA6FF",
    "red":    "#FF6666",
    "green":  "#66FF99",
}


class TaskPanel(QWidget):
    """
    Manages a 3-attempt pick-place sorting task.
    - Start Task
    - For each attempt:
        - "Mark Attempt End" (adds timestamp markers)
        - Select response via radio buttons and flags; enter reasoning
        - "Save Attempt"
        - "Next Attempt"
    - After 3rd attempt:
        - Enter final wrong_sorting + reasoning
        - "Finalize Sorting"
    All logs dumped as JSON into experiment logs folder.
    """
    task_end_signal = Signal(name="task_end")
    task_status_signal = Signal(name="task_status")
    selections_signal = Signal(name="selections_update")
    def __init__(self, camera: CameraPanel, exp: ExperimentManager, parent=None, context=None):
        super().__init__(parent)
        
        self.camera = camera
        self.exp = exp
        self.context = context

        # ros node
        self.ros_node = RosNode(
            context=self.context,
            status_signal=self.task_status_signal,
            selections_signal=self.selections_signal
        )
        self.ros_node_thread = RosThread(context=self.context, node=self.ros_node)
        self.ros_node_thread.start()

        self.task_status_signal.connect(self._task_status_update)

         # --- State & Control Buttons ---
        self.lbl_state = QLabel("Idle")
        self.lbl_state.setMinimumWidth(200)
        self.lbl_state.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        self.btn_start = QPushButton("Start Task")
        self.btn_continue = QPushButton("Continue")
        self.btn_start_sorting = QPushButton("Start Sorting")

        # hide continue button and start sorting initially
        # self.btn_continue.hide()
        # self.btn_start_sorting.hide()

        # --- Attempt tracking ---
        self.attempt_idx = 1
        self.attempts = []
        self.attempt_end_times = []

        # --- Response controls ---
        self.chk_pickplace_failed = QCheckBox("pickplace_failed")
        self.chk_unsafe = QCheckBox("unsafe")
        self.txt_reason = QTextEdit()
        self.txt_reason.setPlaceholderText("Reasoning for this attempt...")

        # --- Final sorting fields ---
        self.chk_wrong_sorting = QCheckBox("wrong_sorting")
        self.txt_sort_reason = QTextEdit()
        self.txt_sort_reason.setPlaceholderText("Final sorting reasoning...")

        # --- Detections box ---
        self.tbl_selections = QTableWidget()
        self.tbl_selections.setColumnCount(4)
        self.tbl_selections.setHorizontalHeaderLabels(["Object", "Color", "Bin Color", "Bin Side"])
        self.tbl_selections.setAlternatingRowColors(True)
        self.tbl_selections.setShowGrid(True)
        # transparent background
        self.tbl_selections.setStyleSheet("background: transparent;")
        self.brn_redo_detections = QPushButton("Redo Detections")

        v_detections = QVBoxLayout()
        v_detections.addWidget(self.tbl_selections)
        v_detections.addWidget(self.brn_redo_detections, alignment=Qt.AlignmentFlag.AlignRight)
        detections_box = QGroupBox("Current Detections")
        detections_box.setLayout(v_detections)

        # --- Attempt form ---
        form_attempt = QFormLayout()
        form_attempt.addRow("", self._row(self.chk_pickplace_failed, self.chk_unsafe))
        form_attempt.addRow("Reasoning:", self.txt_reason)

        self.box_attempt = QGroupBox(f"Pick-Place Attempt #{self.attempt_idx}")
        v_attempt = QVBoxLayout()
        v_attempt.addLayout(form_attempt)
        self.box_attempt.setLayout(v_attempt)

        # --- Final sorting form ---
        form_final = QFormLayout()
        form_final.addRow("", self.chk_wrong_sorting)
        form_final.addRow("Reasoning:", self.txt_sort_reason)

        box_final = QGroupBox("Final Sorting Response")
        v_final = QVBoxLayout()
        v_final.addLayout(form_final)
        box_final.setLayout(v_final)

        # --- Top control bar ---
        row_btns = QHBoxLayout()
        row_btns.addWidget(self.btn_start)
        row_btns.addWidget(self.lbl_state)
        row_btns.addSpacing(20)
        row_btns.addWidget(self.btn_start_sorting)
        row_btns.addWidget(self.btn_continue)
        row_btns.addStretch(1)

        # --- Main attempt area ---
        attemptLay = QHBoxLayout()
        attemptLay.addWidget(self.box_attempt, stretch=1)
        attemptLay.addWidget(box_final, stretch=1)

        # --- Scroll area for forms (so they never hide other elements) ---
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.addWidget(detections_box)
        scroll_layout.addLayout(attemptLay)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(scroll_content)

        # --- Root layout (fixed top bar, scrollable rest) ---
        root = QVBoxLayout(self)
        root.addLayout(row_btns)
        root.addWidget(scroll)

        # Connections
        self.btn_start.clicked.connect(self._toggle_task)
        self.btn_continue.clicked.connect(self._continue_attempt)
        self.brn_redo_detections.clicked.connect(self._send_redo_detections)
        self.btn_start_sorting.clicked.connect(self._send_start_sorting)
        self.selections_signal.connect(self._update_detections)

    def close(self):
        self.ros_node_thread.stop()
        self.ros_node_thread.join(timeout=2)
        return super().close()

    def _task_status_update(self):
        self.lbl_state.setText(f"Task status: {self.ros_node.sorting_task_status.value}")
        if self.ros_node.sorting_task_status == TaskStatus.WAITING:
            self.btn_continue.setEnabled(True)

    def _update_detections(self):
        data = self.ros_node.selections
        if not data:
            self.tbl_selections.setRowCount(0)
            return

        self.tbl_selections.setRowCount(len(data))

        for row, obj in enumerate(data):
            object_class = obj.get("object_class", "unknown").title()
            color_label  = obj.get("color_label", "unknown").title()
            bin_info     = obj.get("bin", {})

            bin_color = bin_info.get("color", "").title()
            bin_side  = f"{bin_info.get('perspective','').title()} {bin_info.get('side','')}"

            # object class
            self.tbl_selections.setItem(row, 0, QTableWidgetItem(object_class))

            # color label (background)
            color_item = QTableWidgetItem(color_label)
            color_hex = COLOR_MAP.get(color_label.lower(), "#FFFFFF")
            color_item.setForeground(QtGui.QColor(color_hex))
            self.tbl_selections.setItem(row, 1, color_item)

            # bin color (background)
            bin_color_item = QTableWidgetItem(bin_color)
            bin_color_hex = COLOR_MAP.get(bin_color.lower(), "#FFFFFF")
            bin_color_item.setForeground(QtGui.QColor(bin_color_hex))
            self.tbl_selections.setItem(row, 2, bin_color_item)

            # bin side
            self.tbl_selections.setItem(row, 3, QTableWidgetItem(bin_side))


    def _row(self, *widgets):
        h = QHBoxLayout()
        for w in widgets:
            h.addWidget(w)
        h.addStretch(1)
        return h

    # ------------------ control functions ------------------

    def _send_redo_detections(self):
        self.ros_node.pub_task_control.publish(String(data=TaskControl.REDO_DETECT.value))

    def _send_start(self):
        self.ros_node.pub_task_control.publish(String(data=TaskControl.START.value))
        self.btn_start_sorting.show()

    def _send_stop(self):
        self.ros_node.pub_task_control.publish(String(data=TaskControl.CONTINUE.value))

    def _send_start_sorting(self):
        self.ros_node.pub_task_control.publish(String(data=TaskControl.START_SORTING.value))

    def _send_continue(self, attempt_no: int):
        self.ros_node.pub_task_control.publish(String(data=TaskControl.CONTINUE.value))

    # ------------------ UI Logic ------------------

    def _toggle_task(self):
        """Start or End the task (toggle behavior)."""
        if self.btn_start.text() == "Start Task":
            # --- START TASK ---
            
            if not self.exp.ensure_run():
                QMessageBox.warning(self, "Experiment", "Please select or create an experiment folder first.")
                return

            # Mark start
            self.camera.mark_event("task_start")

            self.attempt_idx = 1
            self.attempts = []
            self.attempt_end_times = []
            self.lbl_state.setText("Task started. Waiting for first pick-place.")
            self.btn_start.setText("End Task")
            self.btn_start.setStyleSheet("background-color: red; color: white;")
            self.btn_continue.setEnabled(False)
            self.btn_continue.show()
            self.box_attempt.setTitle("Pick-Place Attempt #1")
            self._send_start()

            # Start video recording if available
            if self.camera.cap and self.camera.cap.isOpened():
                self.camera.start_recording()
            else:
                QMessageBox.warning(self, "Camera", "Camera stream not available; skipping video recording.")

        else:
            # --- END TASK ---
            self.lbl_state.setText("Task ended. Saving response...")
            self.btn_start.setText("Start Task")
            self.btn_start.setStyleSheet("")
            self.btn_continue.setEnabled(False)
            self._send_stop()

            # Stop video recording
            self.camera.stop_recording()
            self.camera.mark_event("task_end")

            # Compose and save final SortingResponse
            if self.attempts:
                wrong_sorting = self.chk_wrong_sorting.isChecked()
                final_reason = self.txt_sort_reason.toPlainText().strip()
                final_status = "successful" if all(a.status == "successful" for a in self.attempts) and not wrong_sorting else "failure"

                sorting_resp = SortingResponse(
                    status=final_status,
                    block_status=self.attempts,
                    wrong_sorting=wrong_sorting,
                    reasoning=final_reason,
                )
                out = sorting_resp.dict()
                out["attempt_end_times_rel"] = self.attempt_end_times
                out["finalized_at"] = datetime.datetime.now().isoformat()
                run_dir = self.exp.ensure_run()
                self.exp.write_json(run_dir, "sorting_response.json", out)

            # write the selections
            if self.ros_node.selections:
                run_dir = self.exp.ensure_run()
                self.exp.write_json(run_dir, "sorting_selections.json", self.ros_node.selections)

            self._reset_attempt_fields()
            self._reset_sorting_fields()
            self.box_attempt.setTitle("Pick-Place Attempt #1")
            QMessageBox.information(self, "Task", "Task ended and logged successfully.")
            self.lbl_state.setText("Idle")
            self.task_end_signal.emit()

    def _continue_attempt(self):
        """Called when user clicks Continue after a pick-place finishes."""
        self.attempt_idx += 1
        self.lbl_state.setText(f"Attempt {self.attempt_idx} finished.")
        self.box_attempt.setTitle(f"Pick-Place Attempt #{self.attempt_idx}")
        self.camera.mark_event(f"attempt_{self.attempt_idx - 1}_end")

        # record relative timestamp in seconds
        if self.camera.rec_start_monotonic:
            rel_t = datetime.datetime.now().timestamp() - self.camera.rec_start_monotonic
            self.attempt_end_times.append(f"{rel_t:.3f}")

        status = "successful" if not self.chk_pickplace_failed.isChecked() and not self.chk_unsafe.isChecked() else "failure"
        resp = PickPlaceResponse(
            status=status,
            pickplace_failed=self.chk_pickplace_failed.isChecked(),
            unsafe=self.chk_unsafe.isChecked(),
            reasoning=self.txt_reason.toPlainText().strip(),
        )
        self.attempts.append(resp)
        self._reset_attempt_fields()

        if self.attempt_idx <= 3:
            self.btn_continue.setEnabled(False)
            self.lbl_state.setText(f"Waiting for attempt {self.attempt_idx + 1}...")
            self._send_continue(self.attempt_idx + 1)
            self.camera.mark_event(f"attempt_{self.attempt_idx}_start")
            self.box_attempt.setTitle(f"Pick-Place Attempt #{self.attempt_idx + 1}")
        else:
            self.lbl_state.setText("All attempts done.")
            self.btn_continue.hide()

    def _reset_attempt_fields(self):
        """Reset all pick-place input fields."""
        self.chk_pickplace_failed.setChecked(False)
        self.chk_unsafe.setChecked(False)
        self.txt_reason.clear()

    def _reset_sorting_fields(self):
        """Reset final sorting input fields."""
        self.chk_wrong_sorting.setChecked(False)
        self.txt_sort_reason.clear()
