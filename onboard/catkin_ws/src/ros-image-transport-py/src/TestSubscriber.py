#!/usr/bin/env python

import rospy
import image_transport

def callback(image):
    print(f"receive image of shape {image.shape}")

rospy.init_node("my_node_sub")
image_transport.Subscriber("my_topic/image",callback)

while True:
    rospy.sleep(0.3)
    if rospy.is_shutdown():
        break