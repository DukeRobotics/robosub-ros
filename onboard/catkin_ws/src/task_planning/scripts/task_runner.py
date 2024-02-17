#!/usr/bin/env python3

import rospy
import tf2_ros
from interface.controls import ControlsInterface
from interface.cv import CVInterface
# from buoy_task import BuoyTask
from move_tasks import initial_submerge, move_to_pose_local
import task_utils


def main():
    rospy.init_node("task_planning")
    tfBuffer = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(tfBuffer)
    controls = ControlsInterface(tfBuffer)
    cv = CVInterface()

    try:
        _ = tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(), rospy.Duration(15))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to get transform")
        return

    # Fill with the tasks to do
    # For example: tasks = [gate_task(), buoy_task(), octagon_task()]
    tasks = [move_to_pose_local(controls, task_utils.create_pose(0, 0, -0.5, 0, 0, 0))]

    for t in tasks:
        while not t.done and not rospy.is_shutdown():
            t.step()
        if rospy.is_shutdown():
            break


if __name__ == '__main__':
    main()
