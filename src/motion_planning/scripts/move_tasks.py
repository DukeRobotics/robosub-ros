from task import Task
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class MoveToPoseGlobalTask(Task):

	def __init__(self, x, y, z, roll, pitch, yaw):
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw

		self.desired_pose = Pose()
		desired_pose.point.x = x
		desired_pose.point.y = y
		desired_pose.point.z = z
		desired_pose.quaternion = quaternion_from_euler(roll, pitch, yaw)

	def _task_run(self):
		self.publish_desired_pose_global(self.desired_pose)