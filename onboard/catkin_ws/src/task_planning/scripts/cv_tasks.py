import task_utils
import rospy
from interface.CV import CVInterface
from interface.controls import ControlsInterface
from coroutines import task, Yield, Transform
from move_tasks import move_to_pose_global

@task
async def move_to_obj(controls: ControlsInterface, cv: CVInterface, name):
    
    # Get initial obj. location and start a move_to task (assumed global?)
    pose = cv.get_pose(name)
    move_task = move_to_pose_global(controls, pose) # (Not sure if this should be an await? Shouldn't block rest of fxn)
    
    # Until task is completed
    while not (
        controls.state and task_utils.stopped_at_pose(
            controls.state.pose.pose,
            pose,
            controls.state.twist.twist)):
        # Take in CVObject name as input (assuming higher-level external call to update/change the current CVObject, unless it should just be updated continuously?)
        updated_obj = await Yield()
        
        # Send pose to move_global
        pose = cv.get_pose(updated_obj)
        move_task.send(pose)


