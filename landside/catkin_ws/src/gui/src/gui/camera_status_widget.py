from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

import rospy
import resource_retriever as rr


class CameraStatusWidget(QWidget):

    def __init__(self):
        super(CameraStatusWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/CameraStatusWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        rospy.loginfo('Camera Status Widget successfully initialized')

        self.background_colors = {
            "green": "background-color: #8aff92",
            "red": "background-color: #ff7878"
        }

        self.init_gui()

    def init_gui(self):
        mono_text_area = self.mono_camera_text_area
        dai_text_area = self.dai_camera_text_area

        mono_text_area.setText('<i style="color: red;">DISCONNECTED</i>')
        dai_text_area.setText('<i style="color: red;">DISCONNECTED</i>')
        
        return

    def populate_mono_camera_text_area(self):
        # text_area = self.mono_camera_text_area

        pass

    def populate_dai_camera_text_area(self):
        # text_area = self.mono_camera_text_area

        pass
