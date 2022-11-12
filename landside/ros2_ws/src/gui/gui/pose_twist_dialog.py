from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QDialog
from python_qt_binding.QtCore import pyqtSignal

import resource_retriever as rr


class PoseTwistDialog(QDialog):

    pose = pyqtSignal(float, float, float, float, float, float, name='pose_controls')
    twist = pyqtSignal(float, float, float, float, float, float, name='twist_controls')

    def __init__(self):
        super(PoseTwistDialog, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/PoseTwistDialog.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.pose_twist_select.addItems(('Pose', 'Twist'))
        self.pose_twist_select.activated.connect(self.update_label)

        self.update_label()

        self.dialog_button.accepted.connect(self.accept_clicked)
        self.dialog_button.rejected.connect(self.reject)

    def update_label(self):
        self.label.setText(f'Enter {self.pose_twist_select.currentText()}')

    def accept_clicked(self):
        values = (self.x_value.value(),
                  self.y_value.value(),
                  self.z_value.value(),
                  self.roll_value.value(),
                  self.pitch_value.value(),
                  self.yaw_value.value())
        if self.pose_twist_select.currentText() == 'Pose':
            self.pose.emit(*values)
        else:
            self.twist.emit(*values)
        self.accept()
