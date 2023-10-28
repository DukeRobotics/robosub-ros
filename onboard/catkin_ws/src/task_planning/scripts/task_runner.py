#!/usr/bin/env python3

import rospy
import tf
from interface.controls import ControlsInterface
from interface.cv import CVInterface
from buoy_task import BuoyTask

def main():
    rospy.init_node("task_planning")
    listener = tf.TransformListener()
    controls = ControlsInterface(listener)
    cv = CVInterface()

    listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(15))

    # Fill with the tasks to do
    # For example: tasks = [gate_task(), buoy_task(), octagon_task()]
    tasks = []
    for t in tasks:
        while not t.done and not rospy.is_shutdown():
            t.step()
        if rospy.is_shutdown():
            break

if __name__ == '__main__':
    main()
