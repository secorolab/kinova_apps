import sys
import os
import cv2
import threading
import datetime
from typing import Dict

from PySide6.QtCore import Qt, QTimer, QSize
from PySide6.QtGui import QImage, QPixmap, QAction
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QListWidget, QListWidgetItem,
    QAbstractItemView, QPushButton, QLineEdit, QFileDialog, QMessageBox,
    QHBoxLayout, QVBoxLayout, QGroupBox, QFormLayout, QToolBar, QStatusBar
)

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py import TopicMetadata, Info, MetadataIo

import qdarktheme



class RosDiscovery(Node):
    def __init__(self, *, context):
        super().__init__("ros_cam_recorder_discovery", context=context)

    def list_topics(self) -> Dict[str, str]:
        topics = {}
        for name, types in self.get_topic_names_and_types():
            if types:
                topics[name] = types[0]
        return topics


class RosbagRecorder(Node):
    def __init__(self, *, context=None, output_dir: str, topic_type_map: Dict[str, str]):
        super().__init__("rosbag2_recorder_node", context=context)

        output_dir = self._prepare_output_dir(output_dir)
        self.output_dir = output_dir

        # Initialize SequentialWriter
        self.writer = SequentialWriter()
        storage_opts = StorageOptions(uri=output_dir, storage_id="sqlite3")
        conv_opts = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        self.writer.open(storage_opts, conv_opts)

        self._subs = []
        for topic, type_str in topic_type_map.items():
            msg_cls = get_message(type_str)
            topic_md = TopicMetadata(
                id=0,
                name=topic,
                type=type_str,
                serialization_format="cdr",
            )
            self.writer.create_topic(topic_md)

            def make_cb(tn: str):
                def cb(msg):
                    ts = self.get_clock().now().nanoseconds
                    self.writer.write(tn, serialize_message(msg), ts)
                return cb

            sub = self.create_subscription(msg_cls, topic, make_cb(topic), 10)
            self._subs.append(sub)

        self.get_logger().info(f"Recording {len(self._subs)} topics → {output_dir}")

    def _prepare_output_dir(self, output_dir: str) -> str:
        """Ensure output directory path is unique and non-existent before rosbag2 creates it."""
        try:
            if os.path.exists(output_dir):
                if os.listdir(output_dir):
                    base, name = os.path.split(output_dir)
                    suffix = datetime.datetime.now().strftime("_%Y%m%d_%H%M%S_%f")
                    new_dir = os.path.join(base, name + suffix)
                    self.get_logger().warning(f"Existing dir non-empty; switching to {new_dir}")
                    output_dir = new_dir
                else:
                    # Empty dir — remove it, so rosbag2 can recreate
                    os.rmdir(output_dir)
            # IMPORTANT: do NOT call os.makedirs() here;
            # SequentialWriter.open() will create it.
        except Exception as e:
            self.get_logger().error(f"Error preparing output dir: {e}")
            raise
        return output_dir


    def destroy_node(self):
        """Ensure writer finalizes before node destruction."""
        try:
            if hasattr(self, "writer") and self.writer is not None:
                self.get_logger().info("Finalizing rosbag writer...")
                self.writer.close()
                self.writer = None
        except Exception as e:
            self.get_logger().error(f"Error closing writer: {e}")
        return super().destroy_node()


class RecorderThread(threading.Thread):
    def __init__(self, *, context, output_dir: str, topic_type_map: Dict[str, str]):
        super().__init__(daemon=True)
        self.context = context
        self.output_dir = output_dir
        self.topic_type_map = topic_type_map
        self._stop_event = threading.Event()
        self._executor = None
        self._node = None

    def run(self):
        self._node = RosbagRecorder(context=self.context, output_dir=self.output_dir, topic_type_map=self.topic_type_map)
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


class CameraPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.url_input = QLineEdit("http://192.168.0.101:8080/video")
        self.start_btn = QPushButton("Start Feed")
        self.stop_btn = QPushButton("Stop Feed")
        self.stop_btn.setEnabled(False)
        self.view = QLabel("Camera feed")
        self.view.setAlignment(Qt.AlignCenter)
        self.view.setMinimumSize(QSize(480, 320))
        self.view.setScaledContents(True)

        h = QHBoxLayout()
        h.addWidget(self.url_input, 1)
        h.addWidget(self.start_btn)
        h.addWidget(self.stop_btn)

        box = QGroupBox("Camera")
        inner = QVBoxLayout()
        inner.addLayout(h)
        inner.addWidget(self.view)
        box.setLayout(inner)

        lay = QVBoxLayout(self)
        lay.addWidget(box)

        self.cap = None
        self.timer = QTimer(self)
        self.timer.setInterval(30)
        self.timer.timeout.connect(self._update_frame)
        self.start_btn.clicked.connect(self.start)
        self.stop_btn.clicked.connect(self.stop)

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

    def _update_frame(self):
        if not self.cap:
            return
        ok, frame = self.cap.read()
        if not ok:
            return
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        qimg = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
        self.view.setPixmap(QPixmap.fromImage(qimg))

    def close(self):
        self.stop()


class RosPanel(QWidget):
    def __init__(self, *, context, parent=None):
        super().__init__(parent)
        self.context = context

        self.topics_list = QListWidget()
        self.topics_list.setSelectionMode(QAbstractItemView.MultiSelection)
        self.refresh_btn = QPushButton("Refresh Topics")
        self.record_btn = QPushButton("Start Recording")
        self.stop_btn = QPushButton("Stop Recording")
        self.stop_btn.setEnabled(False)

        self.output_dir_edit = QLineEdit(os.path.abspath("./bags"))
        self.browse_btn = QPushButton("Browse...")

        form = QFormLayout()
        form.addRow("Output dir:", self._row(self.output_dir_edit, self.browse_btn))

        btns = QHBoxLayout()
        btns.addWidget(self.refresh_btn)
        btns.addStretch(1)
        btns.addWidget(self.record_btn)
        btns.addWidget(self.stop_btn)

        box = QGroupBox("ROS 2 Topics")
        inner = QVBoxLayout()
        inner.addWidget(self.topics_list, 1)
        inner.addLayout(form)
        inner.addLayout(btns)
        box.setLayout(inner)

        lay = QVBoxLayout(self)
        lay.addWidget(box)

        self.discovery = RosDiscovery(context=self.context)
        self.recorder_thread: RecorderThread | None = None

        self.refresh_btn.clicked.connect(self.refresh_topics)
        self.record_btn.clicked.connect(self.start_recording)
        self.stop_btn.clicked.connect(self.stop_recording)
        self.browse_btn.clicked.connect(self.pick_output_dir)
        self.topics_list.itemSelectionChanged.connect(self._sel_changed)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(1500)
        self._refresh_timer.timeout.connect(self.refresh_topics)
        self._refresh_timer.start()

        self.refresh_topics()

    def _row(self, *widgets):
        h = QHBoxLayout()
        for w in widgets:
            h.addWidget(w)
        return h

    def refresh_topics(self):
        """Refresh the list of topics without losing selection."""
        current_selection = {item.text().split(" ")[0] for item in self.topics_list.selectedItems()}
        topics = self.discovery.list_topics()

        existing_items = {}
        for i in range(self.topics_list.count()):
            item = self.topics_list.item(i)
            topic_name = item.text().split(" ")[0]
            existing_items[topic_name] = item

        # Add new topics
        for topic_name, type_str in topics.items():
            if topic_name not in existing_items:
                item = QListWidgetItem(f"{topic_name} ({type_str})")
                self.topics_list.addItem(item)
                if topic_name in current_selection:
                    item.setSelected(True)
            else:
                # Update type string if changed
                existing_item = existing_items[topic_name]
                if f"{topic_name} ({type_str})" != existing_item.text():
                    existing_item.setText(f"{topic_name} ({type_str})")
                # Keep selection state stable
                existing_item.setSelected(topic_name in current_selection)

        # Remove topics that disappeared
        i = 0
        while i < self.topics_list.count():
            item = self.topics_list.item(i)
            topic_name = item.text().split(" ")[0]
            if topic_name not in topics:
                self.topics_list.takeItem(i)
            else:
                i += 1

        self._sel_changed()


    def _sel_changed(self):
        has_sel = len(self.topics_list.selectedItems()) > 0
        self.record_btn.setEnabled(has_sel and self.recorder_thread is None)

    def pick_output_dir(self):
        d = QFileDialog.getExistingDirectory(self, "Choose output directory", self.output_dir_edit.text())
        if d:
            self.output_dir_edit.setText(d)

    def _selected_topic_map(self) -> Dict[str, str]:
        topics = self.discovery.list_topics()
        res: Dict[str, str] = {}
        for it in self.topics_list.selectedItems():
            name = it.text().split(" ")[0]
            if name in topics:
                res[name] = topics[name]
        return res

    def start_recording(self):
        topics = self._selected_topic_map()
        if not topics:
            QMessageBox.warning(self, "rosbag2", "Select at least one topic.")
            return

        outdir_base = self.output_dir_edit.text().strip()
        if not outdir_base:
            QMessageBox.warning(self, "rosbag2", "Choose an output directory.")
            return

        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        outdir = os.path.join(outdir_base, f"bag_{ts}")
        os.makedirs(outdir, exist_ok=True)

        self.recorder_thread = RecorderThread(context=self.context, output_dir=outdir, topic_type_map=topics)
        self.recorder_thread.start()
        self.record_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def stop_recording(self):
        if self.recorder_thread:
            self.recorder_thread.stop()
            self.recorder_thread.join(timeout=2.0)
            self.recorder_thread = None
        self.stop_btn.setEnabled(False)
        self._sel_changed()

    def close(self):
        self._refresh_timer.stop()
        self.stop_recording()
        self.discovery.destroy_node()


class MainWindow(QMainWindow):
    def __init__(self, *, context):
        super().__init__()
        self.setWindowTitle("Mobile Camera + ROS2 Recorder")
        self.resize(1100, 700)

        self.camera = CameraPanel()
        self.ros = RosPanel(context=context)

        central = QWidget()
        h = QHBoxLayout(central)
        h.addWidget(self.camera, 2)
        h.addWidget(self.ros, 1)
        self.setCentralWidget(central)

        tb = QToolBar("Main")
        tb.setIconSize(QSize(16, 16))
        self.addToolBar(tb)

        act_refresh = QAction("Refresh Topics", self)
        act_refresh.triggered.connect(self.ros.refresh_topics)
        tb.addAction(act_refresh)

        act_stop = QAction("Stop Recording", self)
        act_stop.triggered.connect(self.ros.stop_recording)
        tb.addAction(act_stop)

        self.setStatusBar(QStatusBar())

    def closeEvent(self, e):
        self.camera.close()
        self.ros.close()
        super().closeEvent(e)


def main():
    context = rclpy.context.Context()
    rclpy.init(context=context)

    app = QApplication(sys.argv)
    app.setStyleSheet(qdarktheme.load_stylesheet())
    win = MainWindow(context=context)
    win.show()
    code = app.exec()

    rclpy.shutdown(context=context)
    sys.exit(code)


if __name__ == "__main__":
    main()

