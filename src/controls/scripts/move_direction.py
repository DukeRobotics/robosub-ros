#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion

from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Empty
from mavros_msgs.msg import OverrideRCIn
from controls.msg import MoveWithSpeeds

class MoveToLocalPose:

    NODE_NAME = 'local_pose_movement'
    GCS_ID = 'SYSID_MYGCS'

    LISTENING_TOPIC = 'controls/move'
    OVERRIDE_TOPIC = '/mavros/rc/override'
    STOP_SERVICE = 'controls/stop'
    MAVPARAM_SET_SERVICE = '/mavros/param/set'
    ARMING_SERVICE = '/mavros/cmd/arming'

    INVALID_SPEEDS_MESSAGE = 'Invalid speeds given, ignoring message and stopping movement. Speeds must be between -1 and 1.'

    OVERRIDERC_PUBLISH_RATE = 1     #in Hz

    def __init__(self):
        self._speeds = [0] * 6
        self._pub = rospy.Publisher(self.OVERRIDE_TOPIC, OverrideRCIn, queue_size=10)


    def _on_receive(self, msg):
        if not self._desired_speeds_valid(msg.speeds):
            rospy.logwarn(self.INVALID_SPEEDS_MESSAGE)
            self._stop(None)
            return

        self._speeds = msg.speeds

    def _stop(self, req):
        self._speeds = [0] * 6

    def run(self):
        rospy.init_node(self.NODE_NAME)
        rospy.wait_for_service(self.MAVPARAM_SET_SERVICE)
        rospy.wait_for_service(self.ARMING_SERVICE)

        rospy.Subscriber(self.LISTENING_TOPIC, MoveWithSpeeds, self._on_receive)
        rospy.Service(self.STOP_SERVICE, Empty, self._stop)

        self._set_gcs_id()

        self._arm_robot()

        rate = rospy.Rate(self.OVERRIDERC_PUBLISH_RATE)
        while not rospy.is_shutdown():
            self._move()
            rate.sleep()

    def _set_gcs_id(self):
        '''Set a parameter on the pixhawk to allow rc override commands'''
        set_mavparam = rospy.ServiceProxy(self.MAVPARAM_SET_SERVICE, ParamSet)
        set_mavparam(self.GCS_ID, ParamValue(1, 1))

    def _arm_robot(self):
        arm = rospy.ServiceProxy(self.ARMING_SERVICE, CommandBool)
        arm(True)

    def _move(self):
        '''Generate rc inputs as defined here:
        https://www.ardusub.com/operators-manual/rc-input-and-output.html
        '''
        output = [0] * 8

        # x: forwards
        # y: left
        # z: up
        output[4] = self._speed_to_pwm(self._speeds[0])
        output[5] = self._speed_to_pwm(-self._speeds[1])
        output[2] = self._speed_to_pwm(self._speeds[2])

        # roll
        # pitch
        # yaw
        output[1] = self._speed_to_pwm(self._speeds[3])
        output[0] = self._speed_to_pwm(-self._speeds[4])
        output[3] = self._speed_to_pwm(-self._speeds[5])

        override_message = OverrideRCIn()
        override_message.channels = output
        self._pub.publish(override_message)

    def _speed_to_pwm(self, speed):
        return 1500 + (speed * 500)

    def _desired_speeds_valid(self, speeds):
        if len(speeds) is not 6:
            return False

        for speed in speeds:
            if speed < -1 or speed > 1:
                return False

        return True

if __name__ == '__main__':
    MoveToLocalPose().run()
