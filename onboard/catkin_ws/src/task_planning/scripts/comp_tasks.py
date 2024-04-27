import rospy
import math

from task import Task, task
import move_tasks
import cv_tasks
from utils import geometry_utils

# from interface.controls import Controls
from interface.state import State


@task
async def prequal_task(self: Task) -> Task[None, None, None]:
    """
    Complete the prequalification task by moving to a series of local poses. Returns when the robot is at the final pose
    with zero velocity, within a small tolerance, completing the prequalifiacation task.
    """

    async def move_with_directions(directions, yaw_override='init'):
        for direction in directions:
            # Add zeroes for roll, pitch, and yaw if they are not specified
            if len(direction) == 3:
                direction += (0, 0, 0)

            rospy.loginfo(f"Moving to {direction}")
            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(*direction),
                global_pose_overrides={"yaw": yaw_override, "roll": "init", "pitch": "init"},
                parent=self)
            rospy.loginfo(f"Moved to {direction}")
            rospy.loginfo()

    async def move_to_cv_obj_with_debug(name, stop_dist=0, yaw_override='init'):
        rospy.loginfo(f"Moving to {name}")
        await cv_tasks.move_to_cv_obj(name,
                                      stop_dist=stop_dist,
                                      global_pose_overrides={"yaw": yaw_override, "roll": "init", "pitch": "init"},
                                      parent=self)
        rospy.loginfo(f"Moved to {name}")
        rospy.loginfo()

    directions = [
        (0, 0, -0.7),  # Submerge
        (2.5, 0, -0.2),  # Move up to the gate
        (4.75, 0, -0.2),  # Move through the gate and a few meters forward
    ]
    await move_with_directions(directions)

    # Move towards the buoy glyph
    await move_to_cv_obj_with_debug("buoy_earth_cetus", stop_dist=0.5)

    directions = [
        (0, 2, -0.1),  # Go around to the left side of the marker
        (-3, 0, -0.1),  # Come back in front of the marker
        (0, -1, 0)  # Move back to the center of lane in front of the marker
    ]
    await move_with_directions(directions)

    directions = [
        (0, 0, 0, 0, 0, math.radians(180))  # Yaw 180
    ]
    await move_with_directions(directions, yaw_override=None)

    pose_dict = geometry_utils.parse_pose(State().state.pose.pose)
    new_yaw = pose_dict["yaw"]

    directions = [
        (4.75, 0, -0.2)  # Move back through the gate
    ]
    await move_with_directions(directions, yaw_override=new_yaw)

    # Move to the gate glyph
    await move_to_cv_obj_with_debug("gate_abydos", stop_dist=0.5, yaw_override=new_yaw)

    directions = [
        (0, 0, -0.5),  # Submerge below the gate
        (3, 0, -0.15)  # Move through the gate, back to the starting position
    ]
    await move_with_directions(directions, yaw_override=new_yaw)
