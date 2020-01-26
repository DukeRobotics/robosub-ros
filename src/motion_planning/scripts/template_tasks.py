from task import Task
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
import tf
import task_utils
from task_runner import TaskRunner

class MoveToPoseGlobalTask(Task):
    """Move to pose given in global coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw):
    	self.desired_pose = Pose()
    	self.desired_pose.point.x = x
    	self.desired_pose.point.y = y
    	self.desired_pose.point.z = z
    	self.desired_pose.quaternion = Quaternion(quaternion_from_euler(roll, pitch, yaw))
        super(MoveToPoseGlobalTask, self).__init__()

    def _task_run(self):
    	task_utils.publish_desired_pose_global(self.desired_pose)
        if(task_utils.at_pose(self.desired_pose, TaskRunner.STATE.pose)):
            self.finish()

class MoveToPoseLocalTask(MoveToPoseGlobalTask):
    """Move to pose given in local coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw):
        super(MoveToPoseLocalTask, self).__init__(x, y, z, roll, pitch, yaw)
        self.desired_pose = task_utils.transform(base_link, odom, pose=self.desired_pose)

class HoldPositionTask(Task):
    """Hold position for a given number of seconds."""

    def __init__(self, hold_time=None):
        """
        Parameters:
            hold_time (double): time to hold in seconds. If None or 0, hold indefinitely
        """
        self.hold_time = hold_time

        super(HoldPositionTask, self).__init__()

    def _task_run(self):
        task_utils.publish_desired_pose_global(self.initial_state.pose)
        if(self.hold_time & ((rospy.get_rostime() - self.start_time) > self.seconds_to_hold)):
            self.finish()

class ListTask(Task):
    """Execute a list of tasks in order."""

    def __init__(self, Tasks):
    	self.tasks = Tasks
        self.index = 0
        super(ListTask, self).__init__()

    def _task_run(self):
        if(self.index == len(self.tasks)):
            self.finish()
        elif(self.tasks[self.index].finished):
            self.index += 1
        else:
            self.tasks[self.index].run()