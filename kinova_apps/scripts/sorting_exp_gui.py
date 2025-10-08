import sys
import rclpy
from rclpy.context import Context
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QStatusBar
)
import qdarktheme

from kinova_apps.gui.sorting_panel import TaskPanel
from kinova_apps.gui.experiment_manager import ExperimentManager, ExpertimentsPanel
from kinova_apps.gui.camera_panel import CameraPanel
from kinova_apps.gui.ros_panel import RosPanel


class MainWindow(QMainWindow):
    def __init__(self, *, context):
        super().__init__()
        self.setWindowTitle("Sorting Task Logger")
        self.resize(1280, 800)

        self.exp = ExperimentManager()
        # self.exp.ensure_run()

        # Panels
        self.exp_panel = ExpertimentsPanel(self.exp)
        self.camera = CameraPanel(self.exp)
        self.ros = RosPanel(context=context, exp=self.exp)
        self.task = TaskPanel(self.camera, self.exp)

        central = QWidget()
        main_v = QVBoxLayout(central)

        panels_h = QHBoxLayout()
        panels_h.addWidget(self.camera, 2)
        panels_h.addWidget(self.ros, 2)

        main_v.addWidget(self.exp_panel, 0)
        main_v.addLayout(panels_h)
        main_v.addWidget(self.task, 0)
        self.setCentralWidget(central)

        self.setStatusBar(QStatusBar())

        # self.refresh()
        self.exp_panel.new_exp_signal.connect(self.refresh)
        self.task.task_end_signal.connect(self.reset)

    def reset(self):
        self.exp_panel.reset()
        self.ros.reset()

    def refresh(self):
        self.camera.refresh()
        self.ros.refresh()

    def closeEvent(self, e):
        self.camera.close()
        self.ros.close()
        super().closeEvent(e)

def main():
    context = Context()
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
