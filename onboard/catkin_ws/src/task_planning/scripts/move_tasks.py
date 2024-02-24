import task_utils
import rospy
from interface.controls import ControlsInterface
from coroutines import task, Yield, Transform


@task
async def move_to_pose_global(controls: ControlsInterface, pose):
    controls.start_new_move()
    controls.publish_desired_position(pose)
    while not task_utils.stopped_at_pose(controls.state.pose.pose, pose, controls.state.twist.twist):
        # Allow users of this task to update the pose
        new_pose = await Yield()
        if new_pose is not None:
            pose = new_pose

        controls.publish_desired_position(pose)


@task
async def move_to_pose_local(controls: ControlsInterface, pose, **kwargs):
    rospy.loginfo("move_to_pose_local started: " + str(pose))
    local_pose = task_utils.local_pose_to_global(controls.tfBuffer, pose)
    await Transform(
        move_to_pose_global(controls, local_pose, level=kwargs['level'] + 1),
        input_transformer=lambda p: task_utils.local_pose_to_global(controls.tfBuffer, p) if p else p)

    rospy.loginfo("move_to_pose_local complete: " + str(pose))


@task
async def move_with_velocity(controls: ControlsInterface, twist):
    controls.start_new_move()
    controls.publish_desired_velocity(twist)
    while not task_utils.at_vel(controls.state.twist.twist, twist):
        new_twist = await Yield()
        if new_twist is not None:
            twist = new_twist

        controls.publish_desired_velocity(twist)


@task
async def move_with_power(controls: ControlsInterface, power, seconds):
    controls.publish_desired_power(power)
    endtime = rospy.time.now() + seconds
    while (rospy.time.now() < endtime):
        new_power = await Yield()
        if new_power is not None:
            power = new_power

        controls.publish_desired_power(power)


@task
async def hold_position(controls: ControlsInterface):
    await move_to_pose_global(controls, controls.state.pose.pose)


# TODO: Need to update this to act through controls instead of setting thruster allocs directly
@task
async def initial_submerge(controls: ControlsInterface, thruster_alloc=0.2, seconds=1):
    rospy.loginfo("initial_submerge started")

    now = rospy.Time.now()
    time_end = now + rospy.Duration(secs=seconds)
    while rospy.Time.now() < time_end:
        controls.publish_thruster_allocs(
            bottom_front_left=thruster_alloc,
            bottom_front_right=thruster_alloc,
            bottom_back_left=thruster_alloc,
            bottom_back_right=thruster_alloc)

        await Yield()

    rospy.loginfo("initial_submerge complete")


@task
async def prequal_task(controls: ControlsInterface):
    directions = [
        (0, 0, -1),
        (12, 0, 0),
        (0, 0.5, 0),
        (1, 0, 0),
        (0, -1, 0),
        (-1, 0, 0),
        (0, 0.5, 0),
        (-12, 0, 0)
    ]
    for direction in directions:
        await move_to_pose_local(controls, task_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0))
