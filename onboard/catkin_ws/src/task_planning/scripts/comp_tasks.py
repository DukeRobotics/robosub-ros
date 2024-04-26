import rospy
import math

from task import Task, task
import move_tasks
import cv_tasks
from utils import geometry_utils

# from interface.controls import Controls
# from interface.state import State


@task
async def prequal_task(self: Task) -> Task[None, None, None]:
    """
    Complete the prequalification task by moving to a series of local poses. Returns when the robot is at the final pose
    with zero velocity, within a small tolerance, completing the prequalifiacation task.
    """

    directions = [
        (0, 0, -0.7),  # Submerge
        (2.5, 0, -0.2),  # Move up to the gate
        (4.75, 0, -0.2),  # Move through the gate and a few meters forward
    ]
    for direction in directions:
        rospy.loginfo(f"Moving to {direction}")
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")
        rospy.loginfo()

    # Move towards the buoy
    cv_task = cv_tasks.move_to_cv_obj("buoy_earth_cetus", parent=self)
    cv_pose = cv_task.send(None)
    while not geometry_utils.at_pose(geometry_utils.create_pose(0, 0, 0, 0, 0, 0), cv_pose, linear_tol=0.5,
                                     angular_tol=0.5):
        cv_pose = cv_task.send(None)
        rospy.loginfo(cv_pose.position)
    rospy.loginfo("Moved to buoy")

    directions = [
        (0, 2, -0.1),  # Go around to the left side of the marker
        (-3, 0, -0.1),  # Come back in front of the marker
        (0, -1, 0),  # Move back to the center of lane in front of the marker
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")

    # Yaw 180 so camera is facing the gate
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, math.pi), parent=self)
    rospy.loginfo("Yawed 180")

    # Move forward until so there is a clear view of the gate glyph
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(4.75, 0, -0.2, 0, 0, 0), parent=self)
    rospy.loginfo("Moved (4.75, 0, -0.2)")

    # Move to the gate glyph
    cv_task = cv_tasks.move_to_cv_obj("gate_abydos", parent=self)
    cv_pose = cv_task.send(None)
    while not geometry_utils.at_pose(geometry_utils.create_pose(0, 0, 0, 0, 0, 0), cv_pose, linear_tol=1,
                                     angular_tol=0.5):
        cv_pose = cv_task.send(None)
        rospy.loginfo(cv_pose.position)
    rospy.loginfo("Moved to gate")

    directions = [
        (0, 0, -0.5),  # Submerge below the gate
        (3, 0, -0.15)  # Move through the gate, back to the starting position
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")
