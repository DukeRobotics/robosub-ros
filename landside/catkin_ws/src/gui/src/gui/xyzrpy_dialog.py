from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QDialog
from python_qt_binding.QtCore import pyqtSignal

import resource_retriever as rr


class XyzRpyDialog(QDialog):

    xyzrpy = pyqtSignal(float, float, float, float, float, float, name='xyzrpy')

    def __init__(self, text):
        super(XyzRpyDialog, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/XyzRpyDialog.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.label.setText(text)

        self.dialog_button.accepted.connect(self.accept_clicked)
        self.dialog_button.rejected.connect(self.reject)

    def accept_clicked(self):
        self.xyzrpy.emit(self.x_pose.value(),
                         self.y_pose.value(),
                         self.z_pose.value(),
                         self.roll_pose.value(),
                         self.pitch_pose.value(),
                         self.yaw_pose.value())
        self.accept()
