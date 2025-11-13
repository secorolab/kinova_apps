import os
import threading
import datetime
from typing import Dict

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QListWidget, QListWidgetItem,
    QAbstractItemView, QPushButton, QLineEdit, QFileDialog, QMessageBox, QCheckBox, QFormLayout
)

from kinova_apps.gui.experiment_manager import ExperimentManager
from typing import Optional


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
                    os.rmdir(output_dir)
        except Exception as e:
            self.get_logger().error(f"Error preparing output dir: {e}")
            raise
        return output_dir

    def destroy_node(self):
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


class RosTopicsPanel(QWidget):
    def __init__(self, *, context, exp: ExperimentManager, parent=None, hide_topic_list: bool = False):
        super().__init__(parent)
        self.context = context
        self.exp = exp
        self.hide_topic_list = hide_topic_list

        self.topics_list = QListWidget()
        self.topics_list.setSelectionMode(QAbstractItemView.SelectionMode.MultiSelection)

        self.refresh_btn = QPushButton("Refresh Topics")
        self.record_btn = QPushButton("Start Recording")
        self.stop_btn = QPushButton("Stop Recording")
        self.stop_btn.setEnabled(False)

        self.output_dir_edit = QLineEdit()

        self.chk_all = QCheckBox("Record ALL topics")

        form = QFormLayout()
        form.addRow("Output dir:", self._row(self.output_dir_edit))

        btns = QHBoxLayout()
        btns.addWidget(self.refresh_btn)
        btns.addStretch(1)
        btns.addWidget(self.chk_all)
        btns.addWidget(self.record_btn)
        btns.addWidget(self.stop_btn)

        box = QGroupBox("ROS 2 Topics")
        inner = QVBoxLayout()
        if not self.hide_topic_list:
            inner.addWidget(self.topics_list, 1)
        inner.addLayout(form)
        inner.addLayout(btns)
        box.setLayout(inner)

        lay = QVBoxLayout(self)
        lay.addWidget(box)

        self.discovery = RosDiscovery(context=self.context)
        self.recorder_thread: Optional[RecorderThread] = None

        self.refresh_btn.clicked.connect(self.refresh_topics)
        self.record_btn.clicked.connect(self.start_recording)
        self.stop_btn.clicked.connect(self.stop_recording)
        self.topics_list.itemSelectionChanged.connect(self._sel_changed)
        self.chk_all.stateChanged.connect(self._sel_changed)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(1500)
        self._refresh_timer.timeout.connect(self.refresh_topics)
        self._refresh_timer.start()

        self.refresh_topics()
    
    def refresh(self):
        self.setup_logs()
        self.output_dir_edit.setText(self.bag_output_base())
        self.refresh_topics()

    def reset(self):
        self.stop_recording()
        self.output_dir_edit.clear()
        self.topics_list.clear()
        self.chk_all.setChecked(False)

    def setup_logs(self):
        run_dir = self.exp.ensure_run()
        self.bags_dir = os.path.join(run_dir, "bags")
        os.makedirs(self.bags_dir, exist_ok=True)

    def bag_output_base(self) -> str:
        self.exp.ensure_run()
        assert self.bags_dir is not None
        return self.bags_dir

    def _row(self, *widgets):
        h = QHBoxLayout()
        for w in widgets:
            h.addWidget(w)
        return h

    def refresh_topics(self):
        current_selection = {item.text().split(" ")[0] for item in self.topics_list.selectedItems()}
        topics = self.discovery.list_topics()

        existing_items = {}
        for i in range(self.topics_list.count()):
            item = self.topics_list.item(i)
            topic_name = item.text().split(" ")[0]
            existing_items[topic_name] = item

        for topic_name, type_str in topics.items():
            if topic_name not in existing_items:
                item = QListWidgetItem(f"{topic_name} ({type_str})")
                self.topics_list.addItem(item)
                if topic_name in current_selection:
                    item.setSelected(True)
            else:
                existing_item = existing_items[topic_name]
                if f"{topic_name} ({type_str})" != existing_item.text():
                    existing_item.setText(f"{topic_name} ({type_str})")
                existing_item.setSelected(topic_name in current_selection)

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
        if self.chk_all.isChecked():
            # when recording all, selections mean EXCLUDED topics
            self.record_btn.setEnabled(self.recorder_thread is None)
        else:
            # when not recording all, selections mean INCLUDED topics
            has_sel = len(self.topics_list.selectedItems()) > 0
            self.record_btn.setEnabled(has_sel and self.recorder_thread is None)

    def _selected_topic_map(self) -> Dict[str, str]:
        topics = self.discovery.list_topics()
        res: Dict[str, str] = {}
        for it in self.topics_list.selectedItems():
            name = it.text().split(" ")[0]
            if name in topics:
                res[name] = topics[name]
        return res

    def _all_topics_with_exclusions(self) -> Dict[str, str]:
        """Return all topics minus selected ones (when record-all is checked)."""
        topics = self.discovery.list_topics()
        excludes = {item.text().split(" ")[0] for item in self.topics_list.selectedItems()}
        return {k: v for k, v in topics.items() if k not in excludes}

    def start_recording(self):
        if self.chk_all.isChecked():
            topics = self._all_topics_with_exclusions()
        else:
            topics = self._selected_topic_map()

        if not topics:
            QMessageBox.warning(self, "rosbag2", "No topics selected or available.")
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
        return super().close()
