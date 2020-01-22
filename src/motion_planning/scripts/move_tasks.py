from task import Task
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import tf

class MoveToPoseGlobalTask(Task):

    def __init__(self, x, y, z, roll, pitch, yaw):
    	self.desired_pose = Pose()
    	self.desired_pose.point.x = x
    	self.desired_pose.point.y = y
    	self.desired_pose.point.z = z
    	self.desired_pose.quaternion = quaternion_from_euler(roll, pitch, yaw)

    def _task_run(self):
    	self.publish_desired_pose_global(self.desired_pose)

class MoveToPoseLocalTask(MoveToPoseGlobalTask):

    def __init__(self):
        self.transformListener = tf.TransformListener()
        self.transformed_pose = TransformListener.transformPose(self.desired_pose, self.state.pose)

    def _task_run(self):
        self.publish_desired_pose_global(self.transformed_pose)
