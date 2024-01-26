#!/usr/bin/env python3

import rospy
import tf
from interface.controls import ControlsInterface
from interface.cv import CVInterface
# from buoy_task import BuoyTask
from move_tasks import move_to_pose_local
import task_utils

def main():
    rospy.init_node("task_planning")
    listener = tf.TransformListener()
    controls = ControlsInterface(listener)
    cv = CVInterface()

    listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(15))
    controls.get_thruster_dict()
    # Fill with the tasks to do
    # For example: tasks = [gate_task(), buoy_task(), octagon_task()]
    tasks = [move_to_pose_local(controls, task_utils.create_pose(1, 0, 0, 0, 0, 0))]
    for t in tasks:
        # TODO make sure that yielding all the way back is the norm not something you have to think about
        while not t.done and not rospy.is_shutdown():
            t.step()
        if rospy.is_shutdown():
            break

if __name__ == '__main__':
    main()
