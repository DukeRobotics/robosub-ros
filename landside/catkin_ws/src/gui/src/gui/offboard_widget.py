from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialogButtonBox

import rospy
import resource_retriever as rr

from gui.thruster_dialog import ThrusterDialog
from custom_msgs.msg import ThrusterSpeeds
from custom_msgs.srv import SetServo, StartLaunch, StopLaunch


class OffboardWidget(QWidget):

    def __init__(self):
        super(OffboardWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/OffboardWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.sub = rospy.Subscriber('/offboard/thruster_speeds', ThrusterSpeeds, self.update_thruster)
        self.spin_boxes = [
            self.tfr_value,
            self.tfl_value,
            self.tbr_value,
            self.tbl_value,
            self.bfr_value,
            self.bfl_value,
            self.bbr_value,
            self.bbl_value
        ]

        self.thruster_dialog = ThrusterDialog() 
        self.thruster_tester_button.clicked.connect(self.thruster_dialog.show)

        rospy.wait_for_service('/offboard/set_servo_angle')
        self.servo_buttons = [
            self.servo_button_1,
            self.servo_button_2,
            self.servo_button_3,
            self.servo_button_4,
            self.servo_button_5,
            self.servo_button_6,
            self.servo_button_7,
            self.servo_button_8,
        ]

        for i in range(8):
            self.servo_buttons[i].setText(f'Activate Servo {i + 1}')
            self.servo_buttons[i].clicked.connect(lambda _, index=i: self.servo_clicked(index))

        rospy.wait_for_service('/start_node')
        rospy.wait_for_service('/stop_node')
        self.upload_arduino_button.clicked.connect(self.upload_arduino_code)
    

    def update_thruster(self, speeds):
        for i in range(8):
            self.spin_boxes[i].setValue(speeds.speeds[i])

    def servo_clicked(self, index):
        if self.servo_buttons[index].text() == f'Activate Servo {index + 1}':
            new_angle = 180
            self.servo_buttons[index].setText(f'Deactivate Servo {index + 1}')
        else:
            new_angle = 0
            self.servo_buttons[index].setText(f'Activate Servo {index + 1}')
        set_servo = rospy.ServiceProxy('/offboard/set_servo_angle', SetServo)
        resp = set_servo(index, new_angle)
        if not resp.success:
            rospy.logerr(f'Servo {index + 1} failed')
    
    def upload_arduino_code(self):
        self.upload_arduino_button.setEnabled(False)
        upload_client = rospy.ServiceProxy('/start_node', StartLaunch)
        resp = upload_client('offboard_comms', 'arduino_upload.sh', [], False)
        rospy.sleep(15)
        stop_client = rospy.ServiceProxy('/stop_node', StopLaunch)
        resp = stop_client(resp.pid)
        self.upload_arduino_button.setEnabled(True)
