from typing import Optional

from interface.cv import CV
# from interface.controls import Controls
# from interface.state import State
from move_tasks import move_to_pose_local
from task import task, Yield, Task


@task
async def move_to_cv_obj(self: Task, name: str) -> Task[None, Optional[str], None]:
    """
    Move to the pose of an object detected by CV. Returns when the robot is at the object's pose with zero velocity,
    within a small tolerance.

    Args:
        name: CV class name of the object to move to

    Send:
        CV class name of new object to move to
    """

    # Get initial object location and initialize task to move to it
    pose = CV().get_pose(name)
    move_task = move_to_pose_local(pose)

    # Move until the robot has reached the object's pose
    # TODO: Stop when stopped recieving decisions
    # TODO: Stop within stop_distance (param)
    while not move_task.done:
        # Update object to move to
        updated_obj = await Yield()

        if updated_obj is not None:
            name = updated_obj

        pose = CV().get_pose(name)

        # TODO: Add offset
        move_task.send(pose)
