from task import Task
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
import tf
import task_utils
import rospy

class MoveToPoseGlobalTask(Task):
    """Move to pose given in global coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw, *args, **kwargs):
        super(MoveToPoseGlobalTask, self).__init__(*args, **kwargs)

    	self.desired_pose = Pose()
    	self.desired_pose.position.x = x
    	self.desired_pose.position.y = y
    	self.desired_pose.position.z = z
    	self.desired_pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))

        

    def _on_task_run(self):
    	self.publish_desired_pose_global(self.desired_pose)
        if task_utils.at_pose(self.desired_pose, self.state.pose.pose):
            self.finish()


class MoveToPoseLocalTask(MoveToPoseGlobalTask):
    """Move to pose given in local coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw, *args, **kwargs):
        super(MoveToPoseLocalTask, self).__init__(x, y, z, roll, pitch, yaw, *args, **kwargs)

        self.transformed_pose = task_utils.transform('base_link', 'odom', self.desired_pose)

    def _on_task_run(self):
        self.publish_desired_pose_global(self.transformed_pose)
        if task_utils.at_pose(self.transformed_pose, self.state.pose.pose):
            self.finish()


class HoldPositionTask(Task):
    """Hold position for a given number of seconds."""

    def __init__(self, hold_time=None, *args, **kwargs):
        """
        Parameters:
            hold_time (double): time to hold in seconds. If None or 0, hold indefinitely
        """
        super(HoldPositionTask, self).__init__(*args, **kwargs)
        self.hold_time = hold_time
        

    def _on_task_run(self):
        self.publish_desired_pose_global(self.initial_state.pose.pose)
        if self.hold_time:
            if (rospy.get_rostime() - self.start_time) > self.hold_time:
                self.finish()

