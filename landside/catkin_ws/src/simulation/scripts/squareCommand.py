#!/usr/bin/env python
import time

import rospy
from std_msgs.msg import Float32MultiArray

if __name__ == "__main__":
    pub = rospy.Publisher("/sim/move", Float32MultiArray, queue_size=10)
    rospy.init_node("move")
    start = rospy.get_rostime().secs

    forwards = [1, 1, 1, 1, 0, 0, 0, 0]
    backwards = [-1, -1, -1, -1, 0, 0, 0, 0]
    right = [-1, 1, 1, -1, 0, 0, 0, 0]  # top view
    left = [1, -1, -1, 1, 0, 0, 0, 0]
    dirs = [forwards, right, backwards, left]
    ct = 0
    toPub = forwards
    a_list = []
    while not a_list:
        # now = rospy.get_rostime()
        # rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        # print(now)
        data = Float32MultiArray()

        if rospy.get_rostime().secs - start > 5:
            ct = (ct + 1) % 4
            start = time.time()
        data.data = dirs[ct]
        pub.publish(data)
    data = Float32MultiArray()
    data.data = [0, 0, 0, 0, 0, 0, 0, 0]
    pub.publish(data)
