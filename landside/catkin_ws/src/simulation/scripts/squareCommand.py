#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

if __name__ == "__main__":
    pub = rospy.Publisher("/sim/move", Float32MultiArray, queue_size=10)
    rospy.init_node("sim_move_square")

    # Initialize direction vectors
    forwards = (1, 1, 1, 1, 0, 0, 0, 0)
    backwards = (-1, -1, -1, -1, 0, 0, 0, 0)
    right = (-1, 1, 1, -1, 0, 0, 0, 0)  # top view
    left = (1, -1, -1, 1, 0, 0, 0, 0)
    dirs = (forwards, right, backwards, left)
    ct = 0

    start = rospy.Time.now()

    while not rospy.is_shutdown():
        data = Float32MultiArray()
        if (rospy.Time.now() - start).to_sec() > 5:
            ct = (ct + 1) % 4
            start = rospy.Time.now()
        data.data = dirs[ct]
        pub.publish(data)
