#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool

SERVO_SERVICE = 'servo_control'

def call_servo_control(turn_left):
    rospy.init_node('servo_control_client')
    rospy.wait_for_service(SERVO_SERVICE)
    try:
        servo_control = rospy.ServiceProxy(SERVO_SERVICE, SetBool)
        resp = servo_control(turn_left)
        print("Response from service: " + resp.message)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    call_servo_control(True)