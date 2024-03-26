#!/usr/bin/env python3

import rospy
import tf2_ros

from task import Task, TaskStatus, TaskUpdatePublisher
from interface.controls import Controls
from interface.state import State
from interface.cv import CV
import move_tasks
from utils import geometry_utils


def main():

    main_initialized = False
    rospy.init_node("task_planning")
    bypass = rospy.get_param("~bypass")

    # When rospy is shutdown, if main finished initializing, publish that it has closed
    def publish_close():
        if main_initialized:
            TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, "main", TaskStatus.CLOSED, None)

    rospy.on_shutdown(publish_close)

    # Initialize transform buffer and listener
    tfBuffer = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(tfBuffer)

    # Initialize interfaces
    Controls(bypass)
    state = State(bypass, tfBuffer)
    CV(bypass)

    # Initialize the task update publisher
    TaskUpdatePublisher()

    # Wait one second for all publishers and subscribers to start
    rospy.sleep(1)

    # Ensure transform from odom to base_link is available
    try:
        _ = tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(), rospy.Duration(15))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to get transform")
        return

    # Ensure state is available
    while not state.state:
        pass

    # Main has initialized
    TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, "main", TaskStatus.INITIALIZED, None)
    main_initialized = True

    # Run tasks
    try:
        # Tasks to run
        tasks = [
            move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, -0.5, 0, 0, 0),
                                          parent=Task.MAIN_ID)
        ]

        # Step through tasks, stopping if rospy is shutdown
        rate = rospy.Rate(30)
        for t in tasks:
            while not t.done and not rospy.is_shutdown():
                t.step()
                rate.sleep()
            if rospy.is_shutdown():
                break

    except BaseException as e:

        # Main has errored
        TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, "main", TaskStatus.ERRORED, e)
        raise

    else:

        # Main has returned
        TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, "main", TaskStatus.RETURNED, None)


if __name__ == '__main__':
    main()
