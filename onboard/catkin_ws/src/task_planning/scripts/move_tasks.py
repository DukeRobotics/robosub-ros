from typing import Optional, List, Tuple, Union
import copy

from transforms3d.euler import quat2euler, euler2quat

import rospy
from geometry_msgs.msg import Pose, Twist

from interface.controls import Controls
from interface.state import State
from task import task, Yield, Task
from utils import geometry_utils, coroutine_utils


@task
async def move_to_pose_global(self: Task, pose: Pose, timeout: int = 30) -> Task[None, Optional[Pose], None]:
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
    start_time = rospy.Time.now()
    while not geometry_utils.stopped_at_pose(State().state.pose.pose, pose, State().state.twist.twist):
        # Allow users of this task to update the pose
        new_pose = await Yield()
        if new_pose is not None:
            pose = new_pose

        Controls().publish_desired_position(pose)

        # Check if the timeout has been reached
        if (rospy.Time.now() - start_time).to_sec() > timeout:
            rospy.logwarn("Move to pose timed out")
            return None


@task
async def move_to_pose_local(self: Task, pose: Pose, keep_level=False,
                             timeout: int = 30) -> Task[None, Optional[Pose], None]:
    """
    Move to local pose in the "base_link" frame. Returns when the robot is at the given pose with zero velocity, within
    a small tolerance.

    Args:
        pose: Local pose to move to

    Send:
        New local pose to move to
    """
    global_pose = geometry_utils.local_pose_to_global(State().tfBuffer, pose)

    if keep_level:
        orig_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(
            State().orig_state.pose.pose.orientation))
        euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(global_pose.orientation))
        global_pose.orientation = geometry_utils.transforms3d_quat_to_geometry_quat(
            euler2quat(orig_euler_angles[0], orig_euler_angles[1], euler_angles[2]))

    return await coroutine_utils.transform(
        move_to_pose_global(global_pose, timeout=timeout, parent=self),
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


@task
async def depth_correction(self: Task, desired_depth: float) -> Task[None, None, None]:
    rospy.loginfo(f"State().depth: {State().depth}")
    depth_delta = desired_depth - State().depth
    rospy.loginfo(f"depth_delta: {depth_delta}")

    rospy.loginfo(f"Started depth correction {depth_delta}")
    await move_to_pose_local(
        geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0),
        parent=self)
    rospy.loginfo(f"Finished depth correction {depth_delta}")


@task
async def correct_depth(self: Task, desired_depth: float):
    await depth_correction(desired_depth, parent=self)


@task
async def move_x(self: Task, step=1):
    await move_to_pose_local(geometry_utils.create_pose(step, 0, 0, 0, 0, 0), parent=self)
    rospy.loginfo(f"Moved x {step}")


@task
async def move_y(self: Task, step=1):
    await move_to_pose_local(geometry_utils.create_pose(0, step, 0, 0, 0, 0), parent=self)
    rospy.loginfo(f"Moved y {step}")


Direction = Union[Tuple[float, float, float], Tuple[float, float, float, float, float, float]]
Directions = List[Direction]


@task
async def move_with_directions(self: Task, directions: Directions, correct_yaw=False, correct_depth=False):

    for direction in directions:
        assert len(direction) in [3, 6], "Each tuple in the directions list must be of length 3 or 6. Tuple "
        f"{direction} has length {len(direction)}."

        await move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")

        if correct_yaw:
            await self.parent.correct_yaw()
        if correct_depth:
            await self.parent.correct_depth()
