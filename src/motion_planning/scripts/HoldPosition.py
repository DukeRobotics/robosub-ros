import rospy
from geometry_msgs.msg import Pose, Twist, Vector3

class HoldPosition(Task):
    """Hold robot position and orientation for a given time in seconds. If duration not provided, hold indefinitely"""

    def __init__(self, seconds_to_hold=None):
        self.hold_pose = self.state.pose
        self.hold_twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.seconds_to_hold = seconds_to_hold

    def _task_run(self):
        if (self.seconds_to_hold==None):
            while (True):
                self.publish_desired_pose_global(self.hold_pose)
                self.publish_desired_twist_global(self.hold_twist)
                self.publish_desired_twist_local(self.hold_twist)
        self.hold_start_time = rospy.get_time()
        while (self.seconds_to_hold > (rospy.get_time() - self.hold_start_time)):
            self.publish_desired_pose_global(self.hold_pose)
            self.publish_desired_twist_global(self.hold_twist)
            self.publish_desired_twist_local(self.hold_twist)

        self.finish()
