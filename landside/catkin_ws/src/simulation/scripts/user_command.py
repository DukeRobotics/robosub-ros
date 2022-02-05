#!/usr/bin/env python3

import rospy
from custom_msgs.msg import ThrusterSpeeds

if __name__ == "__main__":
    pub = rospy.Publisher("/offboard/thruster_speeds", ThrusterSpeeds, queue_size=10)
    rospy.init_node("sim_move_square")

    # Initialize direction vectors
    forwards = (127, 127, 127, 127, 0, 0, 0, 0)
    backwards = (-127, -127, -127, -127, 0, 0, 0, 0)
    right = (-127, 127, 127, -127, 0, 0, 0, 0)  # top view
    left = (127, -127, -127, 127, 0, 0, 0, 0)
    dirs = (forwards, right, backwards, left)
    ct = 0

    dir_map = {
        'forwards': forwards,
        'backwards': backwards,
        'right': right,
        'left': left
    }

    start = rospy.Time.now()

    # rospy.loginfo("Enter direction to move: ")
    # user_input = input()
    # if user_input not in dir_map:
    #     rospy.loginfo("Invalid direction: " + user_input)
    #     exit()
    user_input = 'left'
    speeds = dir_map[user_input]
    while not rospy.is_shutdown():
        data = ThrusterSpeeds()
        data.speeds = speeds
        pub.publish(data)
