#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion

from std_srvs.srv import Empty
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Pose
from controls.msg import DesiredStateLocal

class MoveToLocalPose:

    NODE_NAME = 'local_pose_movement'
    LISTENING_TOPIC = 'desired_pose/local'
    OVERRIDE_TOPIC = '/mavros/rc/override'
    STOP_SERVICE = 'controls/stop'

    INVALID_SPEEDS_MESSAGE = 'Invalid speeds given, ignoring message. Speeds must be between 0 and 1.'

    def __init__(self):
        self._desired_local_state = Pose()
        self._desired_relative_speeds = [0] * 6

        self._pub = rospy.Publisher(self.OVERRIDE_TOPIC, OverrideRCIn, queue_size=10)
        

    def _on_receive(self, msg):
        if not self._desired_speeds_valid:
            rospy.logwarn(self.INVALID_SPEEDS_MESSGE)

        self._desired_local_state = msg.state
        self._desired_relative_speeds = msg.speeds

    def _handle_stop(self, req):
        self._desired_local_state = Pose()
        self._desired_relative_speeds = [0] * 6

    def run(self):
        rospy.init_node(self.NODE_NAME)

        rospy.Subscriber(self.LISTENING_TOPIC, DesiredStateLocal, self._on_receive)
        rospy.Service(self.STOP_SERVICE, Empty, self._handle_stop)

        while not rospy.is_shutdown():
            self._move_to_desired_state()

    def _move_to_desired_state(self):
        '''Generate rc inputs as defined here:
        https://www.ardusub.com/operators-manual/rc-input-and-output.html
        '''
        output = [0] * 8
        quat = [self._desired_local_state.orientation.x,
                self._desired_local_state.orientation.y,
                self._desired_local_state.orientation.z,
                self._desired_local_state.orientation.w ]
        rpy = euler_from_quaternion(quat)

        # x: forwards
        # y: left
        # z: up
        output[4] = self._get_pwm_value(self._desired_local_state.position.x, self._desired_relative_speeds[0])
        output[5] = self._get_pwm_value(self._desired_local_state.position.y, self._desired_relative_speeds[1])
        output[2] = self._get_pwm_value(self._desired_local_state.position.z, self._desired_relative_speeds[2])

        # roll
        # pitch
        # yaw
        output[1] = self._get_pwm_value(rpy[0], self._desired_relative_speeds[3])
        output[0] = self._get_pwm_value(rpy[1], self._desired_relative_speeds[4])
        output[3] = self._get_pwm_value(rpy[2], self._desired_relative_speeds[5])

        override_message = OverrideRCIn()
        override_message.channels = output
        self._pub.publish(override_message)

    def _get_pwm_value(self, state_diff, speed):
        pwm_diff = speed * 500
        if state_diff > 0:
            return 1500 + pwm_diff
        elif state_diff < 0:
            return 1500 - pwm_diff
        else:
            return 1500



    def _desired_speeds_valid(self, speeds):
        if len(speeds) is not 6:
            return False

        for speed in speeds:
            if speed < 0 or speed > 1:
                return False

        return True

if __name__ == '__main__':
    MoveToLocalPose().run()
