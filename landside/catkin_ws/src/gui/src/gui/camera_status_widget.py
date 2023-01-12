from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

import rospy
import resource_retriever as rr


class CameraStatusWidget(QWidget):

    def __init__(self, context):
        super(CameraStatusWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CameraStatusWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        rospy.loginfo('Camera Status Widget successfully initialized')
