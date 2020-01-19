import rospy
from geometry_msgs.msg import Pose, Twist, Vector3

class HoldPosition(Task):
    def _task_run(self, seconds_to_hold=None):
        """Hold robot position and orientation for a given time in seconds. If duration not provided, hold indefinitely"""

        self.hold_pose = self.state.pose
        self.hold_twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        if (seconds_to_hold==None):
            while (True):
                self.publish_desired_pose_global(hold_pose)
                self.publish_desired_twist_global(hold_twist)
                self.publish_desired_twist_local(hold_twist)

        self.hold_start_time = rospy.get_time()
        while (seconds_to_hold > (rospy.get_time() - self.hold_start_time)):
            self.publish_desired_pose_global(hold_pose)
            self.publish_desired_twist_global(hold_twist)
            self.publish_desired_twist_local(hold_twist)

        self.finish()
