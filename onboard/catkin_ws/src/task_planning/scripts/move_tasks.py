import task_utils
from interface.controls import ControlsInterface
from coroutines import task, Yield, Transform

@task
async def move_to_pose_global(controls: ControlsInterface, pose):
    controls.start_new_move()
    controls.publish_desired_position(pose)
    while not (
        controls.state and task_utils.stopped_at_pose(
            controls.state.pose.pose,
            pose,
            controls.state.twist.twist)):
        # Allow users of this task to update the pose
        new_pose = await Yield()
        if new_pose is not None:
            pose = new_pose

        controls.publish_desired_position(pose)

@task
async def move_to_pose_local(controls: ControlsInterface, pose):
    local_pose = task_utils.local_pose_to_global(controls.listener, pose)
    await Transform(
        move_to_pose_global(controls, local_pose),
        input_transformer=lambda p: task_utils.local_pose_to_global(controls.listener, p) if p else p)

@task
async def hold_position(controls: ControlsInterface):
    await move_to_pose_global(controls, controls.state.pose.pose)

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