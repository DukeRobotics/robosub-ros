from interface.cv import CVInterface
from interface.controls import ControlsInterface
from coroutines import task, Yield
from move_tasks import move_to_pose_local


@task
async def move_to_cv_obj(controls: ControlsInterface, cv: CVInterface, name, offset=None, stop_distance=None):

    # Get initial obj. location and start a move_to task (assumed global?)
    pose = cv.get_pose(name)
    move_task = move_to_pose_local(controls, pose)  # (Not sure if this should be an await? Shouldn't block rest of fxn)

    # Until task is completed
    # TODO: Stop when stopped recieving decisions
    # TODO: Stop within stop_distance (param)
    while not move_task.done:
        # Take in CVObject name as input (assuming higher-level external call to update/change the current CVObject,
        # unless it should just be updated continuously?)
        updated_obj = await Yield()

        # Send pose to move_global
        if updated_obj is not None:
            name = updated_obj

        pose = cv.get_pose(name)
        # TODO: Add offset
        move_task.send(pose)
