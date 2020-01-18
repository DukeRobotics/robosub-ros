import rospy
from task import Task
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class MoveRelative(Task):

    DESIRED_POSE_TOPIC = 'controls/desired_pose_global'
    NODE_NAME = 'moveTask'  # TBD

    def __init__(self, x, y, z, orientation_x, orientation_y, orientation_z, orientation_w):
        Task.__init__(self)
        self.x = x
        self.y = y
        self.z = z
        self.orientation_x = orientation_x
        self.orientation_y = orientation_y
        self.orientation_z = orientation_z
        self.orientation_w = orientation_w

    def _initialize(self):
        rospy.init_node(self.NODE_NAME, anonymous=True)
        Task._initialize(self)
        self.posePublisher = rospy.Publisher(self.DESIRED_POSE_TOPIC, Pose, queue_size=3)
        rospy.init_node(self.NODE_NAME, anonymous=True)
        self.rate = rospy.Rate(10)

    def _run_task(self):
        self.pose = Pose()
        #pose.header.stamp = rospy.Time.now()
        #pose.header.frame_id = ""
        self.pose.position.x = self.x
        self.pose.position.y = self.y
        self.pose.position.z = self.z

        quaternion = quaternion_from_euler(self.orientation_x,
                                            self.orientation_y,
                                            self.orientation_z,
                                            self.orientation_w])

        self.pose.orientation.x = quaternion[0]
        self.pose.orientation.y = quaternion[1]
        self.pose.orientation.z = quaternion[2]
        self.pose.orientation.w = quaternion[3]

        while not rospy.is_shutdown():
            self.posePublisher.publish(self.pose)
            self.rate.sleep()

move = MoveRelative(1, 0, 0, 0, 0, 0, 0)
move._initialize()
move._run_task()
