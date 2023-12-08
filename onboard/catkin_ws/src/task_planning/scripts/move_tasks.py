import task_utils
from coroutines import task, Yield, Transform

@task
async def move_to_pose_global(controls, pose):
    while not (
        controls.state and task_utils.stopped_at_pose(
            controls.state.pose.pose,
            pose,
            controls.state.twist.twist)):
        controls.publish_desired_position(pose)
        # Allow users of this task to update the pose
        new_pose = await Yield()
        if new_pose is not None:
            pose = new_pose

@task
async def move_to_pose_local(controls, pose):
    local_pose = task_utils.local_pose_to_global(controls.listener, pose)
    await Transform(
        move_to_pose_global(controls, local_pose),
        input_transformer=lambda p: task_utils.local_pose_to_global(controls.listener, p))

@task
async def hold_position(controls):
    await move_to_pose_global(controls, controls.state.pose.pose)
