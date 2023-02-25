from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer

import rospy
import rosgraph
import rostopic
import resource_retriever as rr

from custom_msgs.msg import SystemUsage


class SystemUsageWidget(QWidget):

    SYSTEM_USAGE_TOPIC = '/system/usage'

    def __init__(self):
        super(SystemUsageWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/SystemUsageWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.cpu_value.setDigitCount(4)
        self.cpu_value.display(0)
        self.ram_value.setDigitCount(4)
        self.ram_value.display(0)
        self.system_sub = None

        self.last_system_time = rospy.Time.now() - rospy.Duration(5)
        self.system_sub_timer = QTimer(self)
        self.system_sub_timer.timeout.connect(self.check_system_topic)
        self.system_sub_timer.start(100)

        rospy.loginfo('System Usage Widget successfully initialized')

    def update_cpu_ram(self, system_usage):
        self.last_system_time = rospy.Time.now()
        self.cpu_value.display(system_usage.cpu_percent)
        self.ram_value.display(system_usage.ram.used)

    def check_system_topic(self):
        self.check_system_sub()
        self.check_system_publisher()

    def check_system_sub(self):
        enabled = rospy.Time.now() - self.last_system_time < rospy.Duration(2)
        self.cpu_value.setEnabled(enabled)
        self.ram_value.setEnabled(enabled)

    def check_system_publisher(self):
        master = rosgraph.Master('/rostopic')
        pubs, _ = rostopic.get_topic_list(master=master)
        for topic_name, _, publishing_nodes in pubs:
            if topic_name == self.SYSTEM_USAGE_TOPIC and len(publishing_nodes) > 0:
                if self.system_sub is None:
                    self.system_sub = rospy.Subscriber(self.SYSTEM_USAGE_TOPIC, SystemUsage, self.update_cpu_ram)
                return

        if self.system_sub is not None:
            self.system_sub.unregister()
            self.system_sub = None

    def close(self):
        self.system_sub_timer.stop()
        self.system_sub.unregister()
