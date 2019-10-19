#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion

from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Empty    # Note: check out this...
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
from controls.msg import MoveWithSpeeds

class DirectionsListener:

    NODE_NAME = 'local_pose_movement'
    GCS_ID = 'SYSID_MYGCS'

    # Listening topics
    LT_X = 'controls/move/x'
    LT_Y = 'controls/move/y'
    LT_Z = 'controls/move/z'
    LT_ROLL = 'controls/move/roll'
    LT_PITCH = 'controls/move/pitch'
    LT_YAW = 'controls/move/yaw'

    OVERRIDE_TOPIC = '/mavros/rc/override'
    STOP_SERVICE = 'controls/stop'
    MAVPARAM_SET_SERVICE = '/mavros/param/set'
    ARMING_SERVICE = '/mavros/cmd/arming'

    INVALID_SPEEDS_MESSAGE = 'Invalid speeds given, ignoring message and stopping movement. Speeds must be between -1 and 1.'

    CHAN_NOCHANGE = 65535

    OVERRIDERC_PUBLISH_RATE = 10     #in Hz
    RESET_SPEEDS_TIME = 0.1  # seconds

    def __init__(self):
        # self._speeds = [0] * 6
        self._pub = rospy.Publisher(self.OVERRIDE_TOPIC, OverrideRCIn, queue_size=10)
        self.fresh = {'x': False, 'y': False, 'z': False, 'roll': False, 'pitch': False, 'yaw': False}
        # self._reset_speeds_duration = rospy.Duration(self.RESET_SPEEDS_TIME)

    def _init_subscribers(self):
        rospy.Subscriber(self.LT_X, Float64, self._on_x)
        rospy.Subscriber(self.LT_Y, Float64, self._on_y)
        rospy.Subscriber(self.LT_Z, Float64, self._on_z)
        rospy.Subscriber(self.LT_ROLL, Float64, self._on_roll)
        rospy.Subscriber(self.LT_PITCH, Float64, self._on_pitch)
        rospy.Subscriber(self.LT_YAW, Float64, self._on_yaw)

    def _on_x(self, x_speed):
        self.fresh['x'] = True
        if not self._speed_valid(x_speed.data):
            return
        output = self._no_change_channels()
        output[4] = self._speed_to_pwm(x_speed.data)
        self._pub.publish(output)

    def _on_y(self, y_speed):
        self.fresh['y'] = True
        if not self._speed_valid(y_speed.data):
            return
        output = self._no_change_channels()
        output[5] = self._speed_to_pwm(-y_speed.data)
        self._pub.publish(output)

    def _on_z(self, z_speed):
        self.fresh['z'] = True
        if not self._speed_valid(z_speed.data):
            return
        output = self._no_change_channels()
        output[2] = self._speed_to_pwm(z_speed.data)
        self._pub.publish(output)

    def _on_roll(self, roll_speed):
        self.fresh['roll'] = True
        if not self._speed_valid(roll_speed.data):
            return
        output = self._no_change_channels()
        output[1] = self._speed_to_pwm(roll_speed.data)
        self._pub.publish(output)

    def _on_pitch(self, pitch_speed):
        self.fresh['pitch'] = True
        if not self._speed_valid(pitch_speed.data):
            return
        output = self._no_change_channels()
        output[0] = self._speed_to_pwm(-pitch_speed.data)
        self._pub.publish(output)

    def _on_yaw(self, yaw_speed):
        self.fresh['yaw'] = True
        if not self._speed_valid(yaw_speed.data):
            return
        output = self._no_change_channels()
        output[3] = self._speed_to_pwm(-yaw_speed.data)
        self._pub.publish(output)

    def _no_change_channels(self):
        return [self.CHAN_NOCHANGE] * 8

    # def _on_receive(self, msg):
    #     if not self._desired_speeds_valid(msg.speeds):
    #         rospy.logwarn(self.INVALID_SPEEDS_MESSAGE)
    #         self._stop(None)
    #         return
    #
    #     self._speeds = msg.speeds
    #     self._time_speeds_received = rospy.Time.now()

    # def _stop(self, req):
    #     self._speeds = [0] * 6

    def run(self):
        rospy.init_node(self.NODE_NAME)
        rospy.wait_for_service(self.MAVPARAM_SET_SERVICE)
        rospy.wait_for_service(self.ARMING_SERVICE)

        # rospy.Subscriber(self.LISTENING_TOPIC, MoveWithSpeeds, self._on_receive)
        # rospy.Service(self.STOP_SERVICE, Empty, self._stop)

        self._set_gcs_id()
        self._arm_robot()

        # self._time_speeds_received = rospy.Time.now()
        self._init_subscribers()

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.fresh['x']:
                self._on_x(Float64(0))
            self.fresh['x'] = False

            if not self.fresh['y']:
                self._on_y(Float64(0))
            self.fresh['y'] = False

            if not self.fresh['z']:
                self._on_z(Float64(0))
            self.fresh['z'] = False

            if not self.fresh['roll']:
                self._on_roll(Float64(0))
            self.fresh['roll'] = False

            if not self.fresh['pitch']:
                self._on_pitch(Float64(0))
            self.fresh['pitch'] = False

            if not self.fresh['yaw']:
                self._on_yaw(Float64(0))
            self.fresh['yaw'] = False

            rate.sleep()

        # rate = rospy.Rate(self.OVERRIDERC_PUBLISH_RATE)
        # while not rospy.is_shutdown():
        #     if rospy.Time.now() - self._time_speeds_received > self._reset_speeds_duration:
        #         self._stop(None)
        #
        #     self._move()
        #     rate.sleep()

    def _set_gcs_id(self):
        '''Set a parameter on the pixhawk to allow rc override commands'''
        set_mavparam = rospy.ServiceProxy(self.MAVPARAM_SET_SERVICE, ParamSet)
        set_mavparam(self.GCS_ID, ParamValue(1, 1))

    def _arm_robot(self):
        arm = rospy.ServiceProxy(self.ARMING_SERVICE, CommandBool)
        arm(True)

    # def _move(self):
    #     '''Generate rc inputs as defined here:
    #     https://www.ardusub.com/operators-manual/rc-input-and-output.html
    #     '''
    #     output = [0] * 8
    #
    #     # x: forwards
    #     # y: left
    #     # z: up
    #     output[4] = self._speed_to_pwm(self._speeds[0])
    #     output[5] = self._speed_to_pwm(-self._speeds[1])
    #     output[2] = self._speed_to_pwm(self._speeds[2])
    #
    #     # roll
    #     # pitch
    #     # yaw
    #     output[1] = self._speed_to_pwm(self._speeds[3])
    #     output[0] = self._speed_to_pwm(-self._speeds[4])
    #     output[3] = self._speed_to_pwm(-self._speeds[5])
    #
    #     override_message = OverrideRCIn()
    #     override_message.channels = output
    #     self._pub.publish(override_message)

    def _speed_valid(self, speed):
        rospy.loginfo(speed)
        return -1.0 <= speed <= 1.0

    def _speed_to_pwm(self, speed):
        return 1500 + (speed * 500)

    # def _desired_speeds_valid(self, speeds):
    #     if len(speeds) is not 6:
    #         return False
    #
    #     for speed in speeds:
    #         if speed < -1 or speed > 1:
    #             return False
    #
    #     return True


if __name__ == '__main__':
    DirectionsListener().run()
