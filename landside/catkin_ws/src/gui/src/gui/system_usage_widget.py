from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer

import rospy
import resource_retriever as rr

from custom_msgs.msg import SystemUsage


class SystemUsageWidget(QWidget):

    def __init__(self, parent):
        super(SystemUsageWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/SystemUsageWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.cpu_value.setDigitCount(4)
        self.cpu_value.display(0)
        self.ram_value.setDigitCount(4)
        self.ram_value.display(0)
        self.system_sub = rospy.Subscriber('/system/usage', SystemUsage, self.update_cpu_ram)

        self.last_system_time = rospy.Time.now() - rospy.Duration(5)
        self.system_sub_timer = QTimer(self)
        self.system_sub_timer.timeout.connect(self.check_system_sub)
        self.system_sub_timer.start(100)

        rospy.loginfo('System Usage Widget successfully initialized')

    def update_cpu_ram(self, system_usage):
        self.last_system_time = rospy.Time.now()
        self.cpu_value.display(system_usage.cpu_percent)
        self.ram_value.display(system_usage.ram.used)

    def check_system_sub(self):
        self.system_usage_box.setEnabled(rospy.Time.now() - self.last_system_time < rospy.Duration(2))
