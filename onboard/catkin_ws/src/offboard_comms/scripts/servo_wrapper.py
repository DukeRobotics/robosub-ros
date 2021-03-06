#!/usr/bin/env python

import rospy
from custom_msgs.msg import ServoAngleArray
from custom_msgs.srv import SetServo


class ServoWrapperPublisher:
    PUBLISHING_TOPIC_SERVO_ANGLES = '/offboard/servo_angles'
    angles = [0,0,0,0,0,0,0,0]
    angle_service_name = 'set_servo_angle'

    def __init__(self):
        rospy.init_node('test_state_publisher')

        self._pub_servo_angles = rospy.Publisher(self.PUBLISHING_TOPIC_SERVO_ANGLES, ServoAngleArray, queue_size=3)
        rospy.Service(self.angle_service_name, SetServo, self.set_servo_angle)

    def set_servo_angle(self, req):
    	if 0 <= req.num <= 7 and 0 <= req.angle <= 180:
    		self.angles[req.num] = req.angle
    		return True
    		
    	return False

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_servo_angles.publish(self.angles)
            rate.sleep()


def main():
    ServoWrapperPublisher().run()


if __name__ == '__main__':
    main()
