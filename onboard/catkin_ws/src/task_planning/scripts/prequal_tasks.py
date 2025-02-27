import rospy
import time
import math

from task import Task, task
import move_tasks
from utils import geometry_utils
from task import Yield

from interface.controls import Controls
from interface.cv import CV
from interface.state import State

RECT_HEIGHT_METERS = 0.3048


@task
async def prequal_task(self: Task) -> Task[None, None, None]:
    """
    Complete the prequalification task by moving to a series of local poses. Returns when the robot is at the final pose
    with zero velocity, within a small tolerance, completing the prequalifiacation task.
    """

    countdown_length = 10
    for i in range(countdown_length):
        rospy.loginfo(f"Starting in {countdown_length - i}")
        start_time = time.time()
        while time.time() - start_time < 1:
            await Yield()
            time.sleep(0.01)

    Controls().call_enable_controls(True)

    DEPTH_LEVEL = -0.5

    await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, DEPTH_LEVEL, 0, 0, 0),
            parent=self)
    rospy.loginfo(f"Moved to (0, 0, {DEPTH_LEVEL})")

    DEPTH_LEVEL = State().depth

    async def rotate_deg(angle):
        rad_angle = math.radians(angle)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, rad_angle),
            parent=self)
        rospy.loginfo(f"Rotate {angle}")

    async def track_blue_rectangle(distance, direction):
        rospy.loginfo(f"track_blue_rectangle {distance} {direction}")
        repeats = math.ceil(distance)
        total_dist = 0
        prev_touching_top = False
        prev_touching_bottom = False
        for i in range(repeats):
            step = distance - total_dist if i == repeats-1 else 1
            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(step * direction, 0, 0, 0, 0, 0),
                parent=self)

            total_dist += step
            rospy.loginfo(f"Moved forward {total_dist}")

            touching_top = CV().cv_data["lane_marker_touching_top"]
            touching_bottom = CV().cv_data["lane_marker_touching_bottom"]
            if touching_top and not touching_bottom:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0.2, 0, 0, 0, 0),
                    parent=self)
                rospy.loginfo("Touching top correction (0, 0.2, 0)")
                if prev_touching_top and not prev_touching_bottom:
                    await rotate_deg(20)
            elif not touching_top and touching_bottom:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, -0.2, 0, 0, 0, 0),
                    parent=self)
                rospy.loginfo("Touching bottom correction (0, -0.2, 0)")
                if not prev_touching_top and prev_touching_bottom:
                    await rotate_deg(-20)

            angle = (CV().cv_data["lane_marker_angle"] * -1)
            if abs(angle) > 0:
                rad_angle = math.radians(angle)
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0, 0, 0, 0, rad_angle),
                    parent=self)
                rospy.loginfo(f"Yaw correction {angle}")

            dist_pixels = CV().cv_data["lane_marker_dist"]
            height_pixels = CV().cv_data["lane_marker_height"]
            dist_meters = dist_pixels * RECT_HEIGHT_METERS / height_pixels
            if abs(dist_meters) > 0:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, dist_meters, 0, 0, 0, 0),
                    parent=self)
                rospy.loginfo(f"Correction {dist_meters}")

            depth_delta = DEPTH_LEVEL - State().depth
            if abs(depth_delta) > 0:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0),
                    parent=self)
                rospy.loginfo(f"Depth correction {depth_delta}")

            prev_touching_top = touching_top
            prev_touching_bottom = touching_bottom

    await track_blue_rectangle(2.5, 1)

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, -0.3, 0, 0, 0),
        parent=self)
    rospy.loginfo("Moved to (0, 0, -0.3)")
    DEPTH_LEVEL = State().depth

    await track_blue_rectangle(2, 1)

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0.3, 0, 0, 0),
        parent=self)
    rospy.loginfo("Moved to (0, 0, 0.3)")
    DEPTH_LEVEL = State().depth

    await track_blue_rectangle(7, 1)

    directions = [
        (1, 0, 0),
        (1, 0, 0),
        (1, 0, 0),
        (0, 1, 0),
        (0, 1, 0),
        (-1, 0, 0),
        (-1, 0, 0),
        (-1, 0, 0),
        (0, -1, 0),
        (0, -0.8, 0),
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")

    await track_blue_rectangle(7, -1)

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, -0.2, 0, 0, 0),
        parent=self)
    rospy.loginfo("Moved to (0, 0, -0.2)")
    DEPTH_LEVEL = State().depth

    await track_blue_rectangle(2, -1)

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0.2, 0, 0, 0),
        parent=self)
    rospy.loginfo("Moved to (0, 0, 0.2)")
    DEPTH_LEVEL = State().depth

    await track_blue_rectangle(3, -1)

    Controls().call_enable_controls(False)
