import rospy
import math

from task import Task, task
import move_tasks
import cv_tasks
from utils import geometry_utils

# from interface.controls import Controls
from interface.state import State
from interface.cv import CV

def get_sign(value):
    if value > 0:
        return 1
    elif value < 0:
        return -1
    else:
        return 0

RECT_HEIGHT_METERS = 0.3048

@task
async def prequal_task(self: Task) -> Task[None, None, None]:
    """
    Complete the prequalification task by moving to a series of local poses. Returns when the robot is at the final pose
    with zero velocity, within a small tolerance, completing the prequalifiacation task.
    """

    await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, -0.5, 0, 0, 0),
            parent=self)
    rospy.loginfo("Moved to (0, 0, -0.5)")

    # angle = 30
    # rad_angle = math.radians(angle)
    # await move_tasks.move_to_pose_local(
    #     geometry_utils.create_pose(0, 0, 0, 0, 0, rad_angle),
    #     parent=self)
    # rospy.loginfo(f"Rotate {angle}")

    # angle = CV().cv_data["blue_rectangle_angle"] * -1
    # rad_angle = math.radians(angle)
    # await move_tasks.move_to_pose_local(
    #     geometry_utils.create_pose(0, 0, 0, 0, 0, rad_angle),
    #     parent=self)
    # rospy.loginfo(f"Rotate {angle}")

    directions = [
        (15, 0, 0),
        (0, 2, 0),
        (-2, 0, 0),
        (0, -1, 0),
        (-15, 0, 0)
        # (0, 0, -0.7),
        # (15, 0, 0),
        # (4.75, 0, 0),
        # (2.5, 0, -0.2),
        # (4.75, 0, -0.2),
        # (4.75, 0, -0.2),
        # (0, 0.6, -0.1),
        # (3.5, 0.1, -0.1),
        # (0, -1.2, -0.1),
        # (-3.5, -0.1, -0.1),
        # (0, 0.6, -0.1),
        # (-7.5, 0, -0.3),
        # (-7.5, 0, -0.3)
    ]
    # for direction in directions:
    #     await move_tasks.move_to_pose_local(
    #         geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
    #         parent=self)
    #     rospy.loginfo(f"Moved to {direction}")

    async def track_blue_rectangle(distance, direction):
        rospy.loginfo(f"track_blue_rectangle {distance} {direction}")
        repeats = math.ceil(distance)
        total_dist = 0
        for i in range(repeats):
            step = distance % 1 if i == repeats-1 else 1
            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(step * direction, 0, 0, 0, 0, 0),
                parent=self)

            total_dist += step
            rospy.loginfo(f"Moved forward {total_dist}")

            angle = CV().cv_data["blue_rectangle_angle"] * -1
            if abs(angle) > 0:
                rad_angle = math.radians(angle)
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0, 0, 0, 0, rad_angle),
                    parent=self)
                rospy.loginfo(f"Rotate {angle}")

            dist_pixels = CV().cv_data["blue_rectangle_dist"]
            height_pixels = CV().cv_data["blue_rectangle_height"]
            dist_meters = dist_pixels * RECT_HEIGHT_METERS / height_pixels
            if abs(dist_meters) > 0:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, dist_meters, 0, 0, 0, 0),
                    parent=self)
                rospy.loginfo(f"Correction {dist_meters}")

    await track_blue_rectangle(2.5, 1)

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, -0.2, 0, 0, 0),
        parent=self)
    rospy.loginfo("Moved to (0, 0, -0.2)")

    await track_blue_rectangle(3, 1)

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0.2, 0, 0, 0),
        parent=self)
    rospy.loginfo("Moved to (0, 0, 0.2)")

    await track_blue_rectangle(7, 1)

    directions = [
        (0, -0.5, 0),
        (1.5, 0, 0),
        (0, 1.5, 0),
        (-1, 0, 0),
        (-1, 0, 0),
        (0, -1, 0),
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")

    await track_blue_rectangle(15, -1)


    # for direction in directions:
    #     x, y, z = direction
    #     repeat = abs(x or y or z)
    #     x = min(x, 1) * get_sign(x)
    #     y = min(y, 1) * get_sign(y)
    #     z = min(z, 1) * get_sign(z)
    #     rospy.loginfo(f"Moving to {(x, y, z)} {repeat} times")
    #     for i in range(repeat):
    #         await move_tasks.move_to_pose_local(
    #             geometry_utils.create_pose(x, y, z, 0, 0, 0),
    #             parent=self)
    #         rospy.loginfo(f"Moved to {direction} * {i}")

    return

    cv_task = cv_tasks.move_to_cv_obj("buoy_earth_cetus", parent=self)
    cv_pose = cv_task.send(None)
    while not geometry_utils.at_pose(geometry_utils.create_pose(0, 0, 0, 0, 0, 0), cv_pose, linear_tol=0.5,
                                     angular_tol=0.5):
        cv_pose = cv_task.send(None)
        rospy.loginfo(cv_pose.position)

    rospy.loginfo("Moved to buoy")

    move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, -math.pi/10, 0, 0), parent=self)
    rospy.loginfo("Rolled -30 deg")

    directions = [
        (0, 2, -0.1),
        (-3, 0, -0.1),
        (0, -1, 0),
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")

    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, math.pi), parent=self)
    rospy.loginfo("Yawed 180")

    await move_tasks.move_to_pose_local(geometry_utils.create_pose(4.75, 0, -0.2, 0, 0, 0), parent=self)
    rospy.loginfo("Moved (4.75, 0, -0.2)")

    cv_task = cv_tasks.move_to_cv_obj("gate_abydos", parent=self)
    cv_pose = cv_task.send(None)
    while not geometry_utils.at_pose(geometry_utils.create_pose(0, 0, 0, 0, 0, 0), cv_pose, linear_tol=1,
                                     angular_tol=0.5):
        cv_pose = cv_task.send(None)
        rospy.loginfo(cv_pose.position)

    rospy.loginfo("Moved to gate")

    move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, -math.pi/10, 0, 0), parent=self)
    rospy.loginfo("Rolled -30 deg")

    directions = [
        (0, 0, -0.5),
        (3, 0, -0.15)
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")
