from typing import Optional
import rospy

from interface.cv import CV
from move_tasks import move_to_pose_local
from task import task, Yield, Task
import move_tasks
from utils import geometry_utils


# TODO: this task will likely be depleted once we complete the refactoring tasks in comp_tasks.py
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
    move_task = move_to_pose_local(pose, parent=self)
    move_task.send(None)

    # Move until the robot has reached the object's pose
    # TODO: Stop when stopped recieving decisions
    # TODO: Stop within stop_distance (param)
    while not move_task.done:
        # Update object to move to
        updated_obj = await Yield(pose)

        if updated_obj is not None:
            name = updated_obj

        pose = CV().get_pose(name)

        # TODO: Add offset
        move_task.send(pose)


@task
async def correct_x(self: Task, prop: str, add_factor: float = 0, mult_factor: float = 1):
    x = (CV().cv_data[prop].coords.x + add_factor) * mult_factor
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(x, 0, 0, 0, 0, 0), parent=self)
    rospy.loginfo(f"Corrected x {x}")


@task
async def correct_y(self: Task, prop: str, add_factor: float = 0, mult_factor: float = 1):
    y = (CV().cv_data[prop].coords.y + add_factor) * mult_factor
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, y, 0, 0, 0, 0), parent=self)
    rospy.loginfo(f"Corrected y {y}")


@task
async def correct_z(self: Task, prop: str):
    z = CV().cv_data[prop].coords.z
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, z, 0, 0, 0), parent=self)
    rospy.loginfo(f"Corrected z {z}")
