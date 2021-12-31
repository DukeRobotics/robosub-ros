from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QDialog
from python_qt_binding.QtCore import pyqtSignal

import rospy
import resource_retriever as rr

class PidDialog(QDialog):

    pid = pyqtSignal(list, list, name='pid')

    def __init__(self):
        super(PidDialog, self).__init__()
        
        ui_file = rr.get_filename('package://gui/resource/PidDialog.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.ppid = [
            (self.x_pos_p, self.x_pos_i, self.x_pos_d),
            (self.y_pos_p, self.y_pos_i, self.y_pos_d),
            (self.z_pos_p, self.z_pos_i, self.z_pos_d),
            (self.roll_pos_p, self.roll_pos_i, self.roll_pos_d),
            (self.pitch_pos_p, self.pitch_pos_i, self.pitch_pos_d),
            (self.yaw_pos_p, self.yaw_pos_i, self.yaw_pos_d)
        ]

        self.vpid = [
            (self.x_twist_p, self.x_twist_i, self.x_twist_d),
            (self.y_twist_p, self.y_twist_i, self.y_twist_d),
            (self.z_twist_p, self.z_twist_i, self.z_twist_d),
            (self.roll_twist_p, self.roll_twist_i, self.roll_twist_d),
            (self.pitch_twist_p, self.pitch_twist_i, self.pitch_twist_d),
            (self.yaw_twist_p, self.yaw_twist_i, self.yaw_twist_d),
        ]

        self.dialog_button.accepted.connect(self.accept_clicked)
        self.dialog_button.rejected.connect(self.reject)

    def show(self, pos_pid, vel_pid):
        for i in range(6):
            for k in range(3):
                self.ppid[i][k].setValue(pos_pid[i][k])
                self.vpid[i][k].setValue(vel_pid[i][k])
        super(PidDialog, self).show()

    def accept_clicked(self):
        pos_pid = [[self.ppid[i][k].value() for k in range(3)]  for i in range(6)]
        vel_pid = [[self.vpid[i][k].value() for k in range(3)] for i in range(6)]
        self.pid.emit(pos_pid, vel_pid)
        self.accept()
