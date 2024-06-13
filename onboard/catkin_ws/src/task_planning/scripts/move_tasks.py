import copy
from typing import Optional

import rospy
from geometry_msgs.msg import Pose, Twist

from interface.controls import Controls
from interface.state import State
from task import task, Yield, Task
from utils import geometry_utils, coroutine_utils


@task
async def move_to_pose_global(self: Task, pose: Pose) -> Task[None, Optional[Pose], None]:
    """
    Move to a global pose in the "odom" frame. Returns when the robot is at the given pose with zero velocity, within
    a small tolerance.

    Args:
        pose: Global pose to move to

    Send:
        New global pose to move to
    """
    Controls().start_new_move()
    Controls().publish_desired_position(pose)
    while not geometry_utils.stopped_at_pose(State().state.pose.pose, pose, State().state.twist.twist):
        # Allow users of this task to update the pose
        new_pose = await Yield()
        if new_pose is not None:
            pose = new_pose

        Controls().publish_desired_position(pose)


@task
async def move_to_pose_local(self: Task, pose: Pose) -> Task[None, Optional[Pose], None]:
    """
    Move to local pose in the "base_link" frame. Returns when the robot is at the given pose with zero velocity, within
    a small tolerance.

    Args:
        pose: Local pose to move to

    Send:
        New local pose to move to
    """
    global_pose = geometry_utils.local_pose_to_global(State().tfBuffer, pose)
    return await coroutine_utils.transform(
        move_to_pose_global(global_pose, parent=self),
        send_transformer=lambda p: geometry_utils.local_pose_to_global(State().tfBuffer, p) if p else p)


@task
async def move_with_velocity(self: Task, twist: Twist) -> Task[None, Optional[Twist], None]:
    """
    Move with a given velocity. Returns when the robot is moving with the given velocity.

    Args:
        twist: Desired velocity

    Send:
        New desired velocity to move with
    """
    Controls().start_new_move()
    Controls().publish_desired_velocity(twist)
    while not geometry_utils.at_vel(State().state.twist.twist, twist):
        new_twist = await Yield()
        if new_twist is not None:
            twist = new_twist

        Controls().publish_desired_velocity(twist)


@task
async def move_with_power_for_seconds(self: Task, power: Twist, seconds: float) -> Task[None, Optional[Twist], None]:
    """
    Move with a given power for a given number of seconds. Returns when the time has elapsed.

    Args:
        power: Desired power
        seconds: Number of seconds to move with the given power

    Send:
        New desired power to move with
    """
    Controls().publish_desired_power(power)
    endtime = rospy.time.now() + seconds
    while (rospy.time.now() < endtime):
        new_power = await Yield()
        if new_power is not None:
            power = new_power

        Controls().publish_desired_power(power)


@task
async def hold_position(self: Task) -> Task[bool, None, None]:
    """
    Hold the position and orientation the robot is at when this task is first run. Does not return.

    Yields:
        If the robot is at the pose it should be holding with zero velocity, within a small tolerance
    """
    pose_to_hold = copy.deepcopy(State().state.pose.pose)
    while True:
        await Yield(geometry_utils.stopped_at_pose(State().state.pose.pose, pose_to_hold, State().state.twist.twist))
        Controls().publish_desired_position(pose_to_hold)
