from typing import Optional

from interface.cv import CVInterface
from interface.controls import ControlsInterface
from interface.state import StateInterface
from move_tasks import move_to_pose_local
from task import task, Yield, Task


@task
async def move_to_cv_obj(self: Task, controls: ControlsInterface, stateInterface: StateInterface, cv: CVInterface,
                         name: str) -> Task[None, Optional[str], None]:

    """
    Move to the pose of an object detected by CV. Returns when the robot is at the object's pose with zero velocity,
    within a small tolerance.

    Args:
        controls: ControlsInterface
        stateInterface: StateInterface
        cv: CVInterface
        name: CV class name of the object to move to

    Send:
        CV class name of new object to move to
    """

    # Get initial object location and initialize task to move to it
    pose = cv.get_pose(name)
    move_task = move_to_pose_local(controls, stateInterface, pose)

    # Move until the robot has reached the object's pose
    # TODO: Stop when stopped recieving decisions
    # TODO: Stop within stop_distance (param)
    while not move_task.done:
        # Update object to move to
        updated_obj = await Yield()

        if updated_obj is not None:
            name = updated_obj

        pose = cv.get_pose(name)

        # TODO: Add offset
        move_task.send(pose)
