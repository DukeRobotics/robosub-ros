import numpy as np
import time

import rospy
from std_msgs.msg import Float32MultiArray
  
if __name__=="__main__":
    pub = rospy.Publisher("move", Float32MultiArray, queue_size=10)
    rospy.init_node("move")
    start = time.time()
    
    forwards = [1,1,1,1,0,0,0,0]
    backwards = [-1,-1,-1,-1,0,0,0,0]
    right = [-1,1,1,-1,0,0,0,0] #top view
    left = [1,-1,-1,1,0,0,0,0]
    dirs = [forwards,right,backwards,left]
    ct = 0
    toPub = forwards
    while True:
        data = Float32MultiArray()
        if (time.time()-start>5):
            ct = (ct+1)%4
            start = time.time()
        data.data = dirs[ct]
        pub.publish(data)