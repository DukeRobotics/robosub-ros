import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion

class SimulationRepublisher():
	SIM_TWIST_TOPIC = 'sim/dvl' #TwistStamped message
	SIM_IMU_TOPIC = 'sim/imu' #Imu message
	SIM_POSE_TOPIC = 'sim/pose' #PoseStamped message
	SENSOR_FUSION_ODOM_TOPIC = 'sensors/dvl/odom'
	SENSOR_FUSION_QUAT_TOPIC = 'sensors/imu/quat'
	RATE = 10 #10Hz

	def __init__(self):
		self.dvl_listener = rospy.Subscriber(self.SIM_TWIST_TOPIC, TwistStamped, self._on_receive_twist)
		self.imu_listener = rospy.Subscriber(self.SIM_IMU_TOPIC, Imu, self._on_receive_imu)
		self.pose_listener = rospy.Subscriber(self.SIM_POSE_TOPIC, PoseStamped, self._on_receive_pose)

		self.odom_pub = rospy.Publisher(self.SENSOR_FUSION_ODOM_TOPIC, Odometry, queue_size=3)
		self.quat_pub = rospy.Publisher(self.SENSOR_FUSION_QUAT_TOPIC, Quaternion, queue_size=3)

		self.odom = Odometry()
		self.quat = Quaternion()
		
		rospy.init_node('simulation_republisher')

	def _on_receive_twist(self, twist_stamped):
		"""Receive TwistStamped message from simulation and assign to odom"""
		self.odom.twist.twist = twist_stamped.twist

	def _on_receive_pose(self, pose_stamped):
		"""Receive PoseStamped message from simulation and assign to odom"""
		self.odom.pose.pose = pose_stamped.pose

	def _on_receive_imu(self, imu):
		"""Receive imu message from simulation and publish quat for sensor fusion"""
		self.quat = imu.orientation

	def republish(self):
		self.rate = rospy.Rate(self.RATE)
		while not rospy.is_shutdown():
			self.odom_pub.publish(self.odom)
			self.quat_pub.publish(self.quat)
			self.rate.sleep()


SimulationRepublisher().republish()