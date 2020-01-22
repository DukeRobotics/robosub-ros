import rospy
from geometry_msgs.msg import Pose, Twist, Vector3

class HoldPosition(Task):
    """Hold robot position and orientation for a given time in seconds. If duration not provided, hold indefinitely"""

    def __init__(self, seconds_to_hold=None):
        self.seconds_to_hold = seconds_to_hold

    def _task_init(self):
        self.hold_twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    def _task_run(self):
        self.publish_desired_pose_global(self.initial_state.pose)
        self.publish_desired_twist_global(self.hold_twist)
        self.publish_desired_twist_local(self.hold_twist)
        if((rospy.get_rostime() - self.start_time) > self.seconds_to_hold):
            self.finish()
