from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import Pose

from interface.cv import CV
# from interface.controls import Controls
# from interface.state import State
from move_tasks import move_to_pose_local
from task import task, Yield, Task

from utils import geometry_utils


@task
async def move_to_cv_obj(self: Task, name: str, stop_dist: float = 0, detection_timeout: float = 1e9,
                         global_pose_overrides: Optional[dict] = None) -> \
                            Task[Tuple[Pose, rospy.Time], Optional[str], Tuple[Pose, rospy.Time]]:
    """
    Move to the pose of an object detected by CV. Returns when the robot is at the object's pose with zero velocity,
    within a small tolerance.

    Args:
        name: CV class name of the object to move to.
        stop_dist: How far away from the object to stop. By default, the robot will never stop moving.
        detection_timeout: How long to wait for a detection before giving up in seconds. By default, the robot will
            wait forever.
        global_pose_overrides: Passed without modification to move_to_pose_local. See move_to_pose_local for more
            information.

    Yields:
        The current pose of the object being moved towards and the timestamp of the most recent detection.

    Send:
        CV class name of new object to move to.

    Returns:
        The final pose of the object moved to and the timestamp of the last obtained detection.
    """

    # Get initial object location, detection timestamp
    pose = CV().get_pose(name)
    timestamp = CV().get_timestamp(name)

    # Begin moving to the object
    move_task = move_to_pose_local(pose, global_pose_overrides=global_pose_overrides, parent=self)
    move_task.send(None)

    # Move until the robot has reached the object's pose
    while geometry_utils.point_norm(pose.position) > stop_dist and \
            timestamp > rospy.Time.now() - rospy.Duration(detection_timeout):
        # Yield object's current pose and timestamp of last detection
        updated_obj = await Yield((pose, timestamp))

        # Update object to move to if a new object is specified
        if updated_obj is not None:
            name = updated_obj

        pose = CV().get_pose(name)
        timestamp = CV().get_timestamp(name)

        move_task.send(pose)

    return pose, timestamp
