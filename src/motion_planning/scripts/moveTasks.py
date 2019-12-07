import rospy
from task import Task
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_from_euler

class MoveRelative(Task):

    DESIRED_POSE_TOPIC = 'poseTopic'  # actual name TBD
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
        Task._initialize(self)
        self.posePublisher = rospy.Publisher(self.DESIRED_POSE_TOPIC, PoseStamped, queue_size=3)
        rospy.init_node(self.NODE_NAME, anonymous=True)
        self.rate = rospy.Rate(10)

    def _run_task(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = ""
        pose.pose.point.x = self.x
        pose.pose.point.y = self.y
        pose.pose.point.z = self.z

        quaternion = quaternion_from_euler([self.orientation_x,
                                            self.orientation_y,
                                            self.orientation_z,
                                            self.orientation_w])
        pose.pose.Quaternion.x = quaternion[0]
        pose.pose.Quaternion.y = quaternion[1]
        pose.pose.Quaternion.z = quaternion[2]
        pose.pose.Quaternion.w = quaternion[3]

        self.posePublisher.publish(pose)
