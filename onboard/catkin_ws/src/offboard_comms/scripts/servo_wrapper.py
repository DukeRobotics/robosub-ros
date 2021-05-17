#!/usr/bin/env python3

import rospy
from custom_msgs.msg import ServoAngleArray
from custom_msgs.srv import SetServo


class ServoWrapperPublisher:
    SERVO_SUB_TOPIC = '/offboard/servo_angles'
    SERVO_SERVICE_TOPIC = '/offboard/set_servo_angle'
    NUM_SERVOS = 8

    def __init__(self):
        rospy.init_node('test_state_publisher')

        self.pub_servo_angles = rospy.Publisher(self.SERVO_SUB_TOPIC, ServoAngleArray, queue_size=3)
        rospy.Service(self.SERVO_SERVICE_TOPIC, SetServo, self.set_servo_angle)

        self.angles = [0 for i in range(self.NUM_SERVOS)]

    def set_servo_angle(self, req):
        if 0 <= req.num < self.NUM_SERVOS and 0 <= req.angle <= 180:
            self.angles[req.num] = req.angle
            return True

        return False

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.pub_servo_angles.publish(self.angles)
            rate.sleep()


def main():
    ServoWrapperPublisher().run()


if __name__ == '__main__':
    main()
