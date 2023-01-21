from python_qt_binding.QtWidgets import (
    QTabWidget,
    QDialog,
    QGridLayout,
)

from gui.camera_status_widget import CameraStatusDataUpdateType


class CameraStatusLog(QDialog):
    def __init__(self, parent):
        dialog = QDialog()

        parent.data_updated.connect(update)

        layout = QGridLayout()

        tabwidget = QTabWidget()
        tabwidget.addTab(self.ping_log_table, "Ping")
        tabwidget.addTab(self.stereo_log_table, "Stereo")
        tabwidget.addTab(self.mono_log_table, "Mono")
        layout.addWidget(tabwidget, 0, 0)

        dialog.setLayout(layout)

        dialog.exec_()

    def update(self, type, status, timestamp):
        CameraStatusDataUpdateType , bool, str
