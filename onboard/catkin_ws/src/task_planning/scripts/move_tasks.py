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
async def move_to_pose_local(self: Task, pose: Pose, global_pose_overrides: Optional[dict] = None) -> \
                             Task[None, Optional[Pose], None]:
    """
    Move to local pose in the "base_link" frame. Returns when the robot is at the given pose with zero velocity, within
    a small tolerance. Below, "local pose" refers to a pose in the "base_link" frame. "Global pose" refers to the local
    pose transformed to the "odom" frame.

    Args:
        pose: Local pose to move to.
        global_pose_overrides: Either a dictionary or None.
            If None (default), it is equivalent to the dictionary {'roll': 'init', 'pitch': 'init'} to maintain the
                robot's initial roll and pitch; typically, this keeps the robot level.
            If a dictionary, all keys must be in list ['x', 'y', 'z', 'roll', 'pitch', 'yaw']. For each key that is
                present, the corresponding axis in the global pose will be overridden with the value of the key. It is
                not necessary for all keys to be present. Each value can be one of three possible types:
                    None: The axis will not be overridden.
                    int or float: The axis will be overridden with this value. Roll, pitch, and yaw values are in
                        radians.
                    'orig': The axis will be overridden with the state of the robot along that axis when the task was
                        started.
                    'init': The axis will be overridden with the state of the robot along that axis when the State
                        interface was initialized.

    Send:
        New local pose to move to.
    """

    # Set default dictionary if global_pose_overrides is None. It keeps the robot level.
    # The default dictionary was not set in the function signature because it is mutable. This would cause the default
    # dictionary to be shared between all calls to the function. If the default dictionary was modified in one call, it
    # would be modified in all calls.
    if global_pose_overrides is None:
        global_pose_overrides = {'roll': 'init', 'pitch': 'init'}

    # Validate the global_pose_overrides dictionary
    for key, value in global_pose_overrides.items():
        if key not in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
            raise ValueError(f"Invalid key in global_pose_overrides: {key}")

        if value is not None and value not in ['orig', 'init'] and not isinstance(value, (int, float)):
            raise ValueError(f"Invalid value in global_pose_overrides: {value} for key: {key}")

    # Remove any keys in global_pose_overrides with a value of None
    global_pose_overrides = {key: value for key, value in global_pose_overrides.items() if value is not None}

    # Get the current pose of the robot when the task was started
    orig_pose = State().state.pose.pose
    orig_pose_dict = geometry_utils.parse_pose(orig_pose)

    # Replace any 'orig' values in global_pose_overrides with the original pose values
    for key, value in global_pose_overrides.items():
        if value == 'orig':
            global_pose_overrides[key] = orig_pose_dict[key]

    # Get the initial pose of the robot when the State interface was initialized
    init_pose = State().init_state.pose.pose
    init_pose_dict = geometry_utils.parse_pose(init_pose)

    # Replace any 'init' values in global_pose_overrides with the initial pose values
    for key, value in global_pose_overrides.items():
        if value == 'init':
            global_pose_overrides[key] = init_pose_dict[key]

    def send_transformer(pose):
        nonlocal global_pose_overrides
        if pose:
            # Transform the local pose to a global pose
            pose = geometry_utils.local_pose_to_global(State().tfBuffer, pose)
            pose_dict = geometry_utils.parse_pose(pose)

            # Override the global pose dict with the values in global_pose_overrides
            pose_dict.update(global_pose_overrides)

            # Convert updated global pose dict back to a Pose message
            pose = geometry_utils.parse_pose_dict(pose_dict)
        return pose

    global_pose = geometry_utils.local_pose_to_global(State().tfBuffer, pose)
    return await coroutine_utils.transform(move_to_pose_global(global_pose, parent=self),
                                           send_transformer=send_transformer)


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
