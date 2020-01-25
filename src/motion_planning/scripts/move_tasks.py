from task import Task
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import tf
import task_utils


class MoveToPoseGlobalTask(Task):
    """Move to pose given in global coordinates."""
    
    def __init__(self, x, y, z, roll, pitch, yaw):
    	self.desired_pose = Pose()
    	self.desired_pose.point.x = x
    	self.desired_pose.point.y = y
    	self.desired_pose.point.z = z
    	self.desired_pose.quaternion = quaternion_from_euler(roll, pitch, yaw)

    def _task_run(self):
    	self.publish_desired_pose_global(self.desired_pose)
        if(task_utils.at_pose(self.desired_pose, self.state.pose)):
            self.finish()

class MoveToPoseLocalTask(MoveToPoseGlobalTask):
    """Move to pose given in local coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw):
        super().__init__(x, y, z, roll, pitch, yaw)
        self.transformListener = tf.TransformListener()
        self.transformed_pose = TransformListener.transformPose(self.desired_pose, self.state.pose) #arguments are incorrect, should call transform function of task_utils.py

    def _task_run(self):
        self.publish_desired_pose_global(self.transformed_pose)
        if(task_utils.at_pose(self.transformed_pose, self.state.pose)):
            self.finish()

class HoldPositionTask(Task):
    """Hold position for a given number of seconds."""

    def __init__(self, hold_time=None):
        """
        Parameters:
            hold_time (double): time to hold in seconds. If None or 0, hold indefinitely
        """
        self.hold_time = hold_time

    def _task_run(self):
        self.publish_desired_pose_global(self.initial_state.pose)
        if(self.hold_time & ((rospy.get_rostime() - self.start_time) > self.seconds_to_hold)):
            self.finish()
