import rospy
from task import Task
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class MoveGlobal(Task):

    DESIRED_POSE_TOPIC = 'controls/desired_pose_global'
    NODE_NAME = 'moveTask'  # TBD

    def __init__(self, x, y, z, roll, pitch, yaw):
        Task.__init__(self)
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def _initialize(self):
        rospy.init_node(self.NODE_NAME, anonymous=True)
        Task._initialize(self)
        self.posePublisher = rospy.Publisher(self.DESIRED_POSE_TOPIC, Pose, queue_size=3)
        rospy.init_node(self.NODE_NAME, anonymous=True)
        self.rate = rospy.Rate(10)

    def _run_task(self):
        self.pose = Pose()
        self.pose.position.x = self.x
        self.pose.position.y = self.y
        self.pose.position.z = self.z

        quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)

        self.pose.orientation.x = quaternion[0]
        self.pose.orientation.y = quaternion[1]
        self.pose.orientation.z = quaternion[2]
        self.pose.orientation.w = quaternion[3]

        while not rospy.is_shutdown():
            self.posePublisher.publish(self.pose)
            self.rate.sleep()

move = MoveGlobal(1, 0, 0, 0, 0, 0)
move._initialize()
move._run_task()
