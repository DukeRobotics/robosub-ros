from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer

import rospy
import resource_retriever as rr
import rosservice

from gui.thruster_dialog import ThrusterDialog
from custom_msgs.msg import ThrusterSpeeds
from custom_msgs.srv import SetServo, StartLaunch, StopLaunch


class OffboardWidget(QWidget):

    def __init__(self):
        super(OffboardWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/OffboardWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

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
        self.sub = rospy.Subscriber('/offboard/thruster_speeds', ThrusterSpeeds, self.update_thruster)

        self.thruster_time = rospy.Time.now() - rospy.Duration(5)
        self.thruster_timer = QTimer(self)
        self.thruster_timer.timeout.connect(self.check_thrusters)
        self.thruster_timer.start(100)

        self.thruster_dialog = ThrusterDialog()
        self.thruster_tester_button.clicked.connect(self.thruster_dialog.show)

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

        self.servo_timer = QTimer(self)
        self.servo_timer.timeout.connect(self.check_servo_service)
        self.servo_timer.start(100)

        self.remote_launch_timer = QTimer(self)
        self.remote_launch_timer.timeout.connect(self.check_remote_launch)
        self.remote_launch_timer.start(100)
        self.upload_arduino_button.clicked.connect(self.upload_arduino_code)

        rospy.loginfo("Offboard Widget successfully initialized")

    def update_thruster(self, speeds):
        self.thruster_time = rospy.Time.now()
        for i in range(8):
            self.spin_boxes[i].setEnabled(True)
            self.spin_boxes[i].setValue(speeds.speeds[i])

    def check_thrusters(self):
        self.thruster_output_box.setEnabled(rospy.Time.now() - self.thruster_time < rospy.Duration(2))

    def check_servo_service(self):
        self.servo_control_box.setEnabled('/offboard/set_servo_angle' in rosservice.get_service_list())

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

    def check_remote_launch(self):
        self.upload_arduino_button.setEnabled('/start_node' in rosservice.get_service_list())

    def upload_arduino_code(self):
        self.upload_arduino_button.setEnabled(False)
        upload_client = rospy.ServiceProxy('/start_node', StartLaunch)
        resp = upload_client('offboard_comms', 'arduino_upload.sh', [], False)
        rospy.sleep(15)
        stop_client = rospy.ServiceProxy('/stop_node', StopLaunch)
        resp = stop_client(resp.pid)
        self.upload_arduino_button.setEnabled(True)
