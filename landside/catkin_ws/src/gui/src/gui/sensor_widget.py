from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer
import python_qt_binding.QtCore as QtCore

import rospy
import resource_retriever as rr
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import rosservice

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped, Twist
from robot_localization.srv import SetPose

from gui.xyzrpy_dialog import XyzRpyDialog


class SensorWidget(QWidget):

    def __init__(self):
        super(SensorWidget, self).__init__()

        ui_file = rr.get_filename('package://gui/resource/SensorWidget.ui', use_protocol=False)
        loadUi(ui_file, self)

        self.labels = {
            'imu' : self.imu_status,
            'dvl' : self.dvl_status,
            'pressure' : self.pressure_status,
            'joystick' : self.joystick_status
        }

        self.times = {
            'imu' : rospy.Time.now() - rospy.Duration(2),
            'dvl' : rospy.Time.now() - rospy.Duration(2),
            'pressure' : rospy.Time.now() - rospy.Duration(2),
            'joystick' : rospy.Time.now() - rospy.Duration(2)
        }

        self.imu_sub = rospy.Subscriber('/sensors/imu/imu', Imu, lambda _: self.update_subs('imu'))
        self.dvl_sub = rospy.Subscriber('/sensors/dvl/odom', Odometry, lambda _: self.update_subs('dvl'))
        self.pressure_sub = rospy.Subscriber('/sensors/depth', Float64, lambda _: self.update_subs('pressure'))
        self.joystick_sub = rospy.Subscriber('/joystick/raw', Joy, lambda _: self.update_subs('joystick'))

        self.timers = {
            'imu' : QTimer(self),
            'dvl' : QTimer(self),
            'pressure' : QTimer(self),
            'joystick' : QTimer(self)   
        }
        for key in self.timers:
            self.timers[key].timeout.connect(lambda k=key: self.update_status(k))
            self.timers[key].start(100)

        self.state_sub = rospy.Subscriber('/state', Odometry, self.update_state)
        self.state_time = rospy.Time.now() - rospy.Duration(5)
        self.state_sub_timer = QTimer(self)
        self.state_sub_timer.timeout.connect(self.update_state_display)
        self.state_sub_timer.start(100)

        self.state_dialog = XyzRpyDialog('Enter Pose')
        self.state_dialog.xyzrpy.connect(self.set_state)
        self.set_state_button.clicked.connect(self.state_dialog.show)
        self.set_state_timer = QTimer(self)
        self.set_state_timer.timeout.connect(self.check_set_state)
        self.set_state_timer.start(100)

        self.background_colors = {
            "green"   : "background-color: #8aff92",
            "red"     : "background-color: #ff7878"
        }

        self.keyboard_timer = QTimer(self)
        self.keyboard_timer.setInterval(100)
        self.keyboard_timer.timeout.connect(self.publish_power)
        self.enable_keyboard_button.clicked.connect(self.enable_keyboard)
        self.enable_keyboard_button.setStyleSheet(self.background_colors['green'])
        self.keyboard_enabled = False
        self.power_publisher = rospy.Publisher('/controls/desired_power', Twist, queue_size=3)
        self.power_msg = Twist()


    def update_subs(self, key):
        self.times[key] = rospy.Time.now()

    def update_status(self, key):
        if rospy.Time.now() - self.times[key] < rospy.Duration(2):
            self.labels[key].setText('CONNECTED')
            self.labels[key].setStyleSheet("QLabel {color : green; }")
        else:
            self.labels[key].setText('DISCONNECTED')
            self.labels[key].setStyleSheet("QLabel {color : red; }")

    def update_state_display(self):
        self.state_box.setEnabled(rospy.Time.now() - self.state_time < rospy.Duration(2))

    def update_state(self, state):
        self.state_time = rospy.Time.now()
        pose = state.pose.pose
        roll, pitch, yaw = euler_from_quaternion((pose.orientation.x, pose.orientation.y, 
                                                  pose.orientation.z, pose.orientation.w))
        self.x_pose.setValue(pose.position.x)
        self.y_pose.setValue(pose.position.y)
        self.z_pose.setValue(pose.position.z)
        self.roll_pose.setValue(np.degrees(roll))
        self.pitch_pose.setValue(np.degrees(pitch))
        self.yaw_pose.setValue(np.degrees(yaw))

        twist = state.twist.twist
        self.x_twist.setValue(twist.linear.x)
        self.y_twist.setValue(twist.linear.y)
        self.z_twist.setValue(twist.linear.z)
        self.roll_twist.setValue(np.degrees(twist.angular.x))
        self.pitch_twist.setValue(np.degrees(twist.angular.y))
        self.yaw_twist.setValue(np.degrees(twist.angular.z))

    def check_set_state(self):
        self.set_state_button.setEnabled('/set_pose' in rosservice.get_service_list())

    def set_state(self, x, y, z, roll, pitch, yaw):
        quat = quaternion_from_euler(np.radians(roll), np.radians(pitch), np.radians(yaw))
        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position = Point(x=x, y=y, z=z)
        pose.pose.pose.orientation = Quaternion(*quat)

        set_pose = rospy.ServiceProxy('/set_pose', SetPose)
        try:
            set_pose(pose)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def publish_power(self):
        self.power_publisher.publish(self.power_msg)
        self.power_msg = Twist()

    def keyPressEvent(self, event):
        if self.keyboard_enabled:
            if event.key() == QtCore.Qt.Key_W:
                self.power_msg.linear.x = 1.0
            if event.key() == QtCore.Qt.Key_S:
                self.power_msg.linear.x = -1.0
            if event.key() == QtCore.Qt.Key_A:
                self.power_msg.linear.y = 1.0
            if event.key() == QtCore.Qt.Key_D:
                self.power_msg.linear.y = -1.0
            if event.key() == QtCore.Qt.Key_I:
                self.power_msg.linear.z = 1.0
            if event.key() == QtCore.Qt.Key_K:
                self.power_msg.linear.z = -1.0
            if event.key() == QtCore.Qt.Key_Right:
                self.power_msg.angular.x = -1.0
            if event.key() == QtCore.Qt.Key_Left:
                self.power_msg.angular.x = 1.0
            if event.key() == QtCore.Qt.Key_Up:
                self.power_msg.angular.y = 1.0
            if event.key() == QtCore.Qt.Key_Down:
                self.power_msg.angular.y = -1.0
            if event.key() == QtCore.Qt.Key_J:
                self.power_msg.angular.z = 1.0
            if event.key() == QtCore.Qt.Key_L:
                self.power_msg.angular.z = -1.0
        super(SensorWidget, self).keyPressEvent(event)

    def enable_keyboard(self):
        if self.keyboard_enabled:
            self.keyboard_timer.stop()
            self.releaseKeyboard()
            self.keyboard_enabled = False
            self.enable_keyboard_button.setStyleSheet(self.background_colors['green'])
            self.enable_keyboard_button.setText('Enable Keyboard Control')
        else:
            self.power_msg = Twist()
            self.grabKeyboard()
            self.keyboard_timer.start()
            self.keyboard_enabled = True
            self.enable_keyboard_button.setStyleSheet(self.background_colors['red'])
            self.enable_keyboard_button.setText('Disable Keyboard Control')
