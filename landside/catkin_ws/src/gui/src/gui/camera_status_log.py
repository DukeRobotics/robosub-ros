from python_qt_binding.QtWidgets import (
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QTabWidget,
    QDialog,
    QGridLayout,
)

from python_qt_binding.QtGui import QColor
from gui.camera_status_widget import CameraStatusDataUpdateType


class CameraStatusLog(QDialog):
    def __init__(self, parent):
        self.mono_log_table = QTableWidget()
        self.stereo_log_table = QTableWidget()
        self.ping_log_table = QTableWidget()

        dialog = QDialog()

        parent.data_updated.connect(self.update)

        layout = QGridLayout()

        tabwidget = QTabWidget()
        tabwidget.addTab(self.ping_log_table, "Ping")
        tabwidget.addTab(self.stereo_log_table, "Stereo")
        tabwidget.addTab(self.mono_log_table, "Mono")
        layout.addWidget(tabwidget, 0, 0)

        dialog.setLayout(layout)

        dialog.exec_()

    def update(self, type, status, timestamp):
        if type == CameraStatusDataUpdateType.PING:
            self.update_ping_table(status, timestamp)
        elif type == CameraStatusDataUpdateType.STEREO:
            self.update_stereo_table(status, timestamp)
        elif type == CameraStatusDataUpdateType.MONO:
            self.update_mono_table(status, timestamp)

    def update_mono_table(self, status, timestamp):
        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        table = self.mono_log_table

        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setColumnCount(2)

        rowPosition = 0
        table.insertRow(rowPosition)

        table.setItem(rowPosition, 0, status_item)
        table.setItem(rowPosition, 1, QTableWidgetItem(timestamp))

    def update_stereo_table(self, status, timestamp):
        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        table = self.stereo_log_table

        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setColumnCount(2)

        rowPosition = 0
        table.insertRow(rowPosition)

        table.setItem(rowPosition, 0, status_item)
        table.setItem(rowPosition, 1, QTableWidgetItem(timestamp))

    def update_ping_table(self, status, timestamp):
        status_msg = "Successful" if status else "Failed"
        color = "green" if status else "red"

        status_item = QTableWidgetItem(status_msg)
        status_item.setForeground(QColor(color))

        table = self.ping_log_table

        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setColumnCount(2)

        rowPosition = 0
        table.insertRow(rowPosition)

        table.setItem(rowPosition, 0, status_item)
        table.setItem(rowPosition, 1, QTableWidgetItem(timestamp))
