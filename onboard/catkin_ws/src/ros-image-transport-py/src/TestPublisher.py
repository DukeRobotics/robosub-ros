#!/usr/bin/env python

import rospy
import numpy as np
import cv2 as cv
import image_transport

rospy.init_node("my_node_pub")
pub = image_transport.Publisher("my_topic")

img = np.zeros((64,64,3),dtype=np.uint8)
img = cv.circle(
    img,
    (32,32),
    10,
    (0,0,255),
    -1
)

while True:
    pub.publish(img)
    print("send !")
    rospy.sleep(0.5)
    if rospy.is_shutdown():
        break