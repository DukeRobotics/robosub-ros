import numpy as np
import time
import thread

import rospy
from std_msgs.msg import Float32MultiArray


def input_thread(a_list):
    raw_input()
    a_list.append(True)

if __name__=="__main__":
    pub = rospy.Publisher("/sim/move", Float32MultiArray, queue_size=10)
    rospy.init_node("move")
    start = time.time()

    forwards = [1,1,1,1,0,0,0,0]
    backwards = [-1,-1,-1,-1,0,0,0,0]
    right = [-1,1,1,-1,0,0,0,0] #top view
    left = [1,-1,-1,1,0,0,0,0]
    dirs = [forwards,right,backwards,left]
    ct = 0
    toPub = forwards
    a_list = []
    thread.start_new_thread(input_thread, (a_list,))
    print("Started sending. Press enter to kill.")
    while not a_list:
        #now = rospy.get_rostime()
        #rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        #print(now)
        data = Float32MultiArray()
        if (time.time()-start>5):
            ct = (ct+1)%4
            start = time.time()
        data.data = dirs[ct]
        pub.publish(data)
    data = Float32MultiArray()
    data.data = [0,0,0,0,0,0,0,0]
    pub.publish(data)
    
