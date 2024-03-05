from task import Task, task
from interface.controls import ControlsInterface
from interface.state import StateInterface
import move_tasks
from utils import geometry_utils


@task
async def prequal_task(self: Task, controls: ControlsInterface, stateInterface: StateInterface) -> \
        Task[None, None, None]:
    """
    Complete the prequalification task by moving to a series of local poses. Returns when the robot is at the final pose
    with zero velocity, within a small tolerance, completing the prequalifiacation task.

    Args:
        controls: ControlsInterface
    """

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
        await move_tasks.move_to_pose_local(
            controls, stateInterface, geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
