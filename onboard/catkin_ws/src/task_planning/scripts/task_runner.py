#!/usr/bin/env python3

import rospy
import tf2_ros
import math
from tqdm import tqdm

from task import Task, TaskStatus, TaskUpdatePublisher
from interface.controls import Controls
from interface.state import State
from interface.cv import CV
import comp_tasks


def main():

    main_initialized = False
    rospy.init_node("task_planning")
    bypass = rospy.get_param("~bypass")
    untethered = rospy.get_param("~untethered")

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
            comp_tasks.initial_submerge(-0.7, parent=Task.MAIN_ID),
            comp_tasks.coin_flip(parent=Task.MAIN_ID),
            comp_tasks.yaw_to_cv_object('gate_red_cw', direction=1, yaw_threshold=math.radians(10),
                                        latency_threshold=1, depth_level=0.7, parent=Task.MAIN_ID),
            comp_tasks.gate_task(offset=-0.1, direction=-1, parent=Task.MAIN_ID),
            comp_tasks.gate_style_task(depth_level=0.9, parent=Task.MAIN_ID),
            comp_tasks.yaw_to_cv_object('buoy', direction=1, depth_level=0.7, parent=Task.MAIN_ID),
            comp_tasks.buoy_task(turn_to_face_buoy=False, depth=0.7, parent=Task.MAIN_ID),
            comp_tasks.after_buoy_task(parent=Task.MAIN_ID)


            # comp_tasks.align_path_marker(direction=-1, parent=Task.MAIN_ID),
            # comp_tasks.center_path_marker(parent=Task.MAIN_ID),
            # comp_tasks.yaw_to_cv_object('path_marker', yaw_threshold=math.radians(5), direction=-1, depth_level=0.5,
            #                             parent=Task.MAIN_ID),
            # comp_tasks.dead_reckoning_path_marker_to_bin(maximum_distance=4, parent=Task.MAIN_ID),
            # comp_tasks.path_marker_to_pink_bin(maximum_distance=6, parent=Task.MAIN_ID),
            # comp_tasks.buoy_to_octagon(direction=1, move_forward=0, parent=Task.MAIN_ID),
            # comp_tasks.let_search_for_bin_turtlesim_style_because_why_not(parent=Task.MAIN_ID),
            # comp_tasks.bin_task(parent=Task.MAIN_ID),
            # comp_tasks.yaw_to_cv_object('bin_pink_front', direction=-1, yaw_threshold=math.radians(15),
            #                             depth_level=1.0, parent=Task.MAIN_ID),
            # comp_tasks.octagon_task(direction=1, parent=Task.MAIN_ID),
        ]
        input("Press enter to run tasks...")

        if untethered:
            rospy.loginfo("Countdown started...")
            for i in tqdm(range(10, 0, -1)):
                rospy.sleep(1)
                if rospy.is_shutdown():
                    break
            Controls().call_enable_controls(True)

        rospy.loginfo("Running tasks.")

        # Step through tasks, stopping if rospy is shutdown
        rate = rospy.Rate(30)
        for t in tasks:
            while not t.done and not rospy.is_shutdown():
                t.step()
                rate.sleep()
            if rospy.is_shutdown():
                break

        if untethered:
            Controls().call_enable_controls(False)

    except BaseException as e:

        # Main has errored
        TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, "main", TaskStatus.ERRORED, e)
        raise

    else:

        # Main has returned
        TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, "main", TaskStatus.RETURNED, None)


if __name__ == '__main__':
    main()
