from task import Task
from geometry_msgs.msg import Pose, Quaternion, Twist, Point, Vector3
from tf.transformations import quaternion_from_euler
import task_utils
import rospy


class MoveToPoseGlobalTask(Task):
    """Move to pose given in global coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw):
        super(MoveToPoseGlobalTask, self).__init__()

        self.desired_pose = Pose()
        self.desired_pose.position = Point(x=x, y=y, z=z)
        self.desired_pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))
        self.desired_twist = Twist()  # default 0 angular and linear velocity

    def _on_task_run(self):
        self.publish_desired_pose_global(self.desired_pose)
        at_desired_pose_vel = task_utils.stopped_at_pose(self.state.pose.pose, self.desired_pose, self.state.twist.twist)
        if at_desired_pose_vel:
            self.finish()

class AllocatePowerTask(Task):
    """Allocate specified power amount in a direction"""

    def __init__(self, x, y, z, roll, pitch, yaw):
        """
        Parameters:
            x (float): x-component of linear velocity
            y (float): y-component of linear velocity
            z (float): z-component of linear velocity
            roll (float): roll-component of angular velocity;
            pitch (float): pitch-component of angular velocity
            yaw (float): yaw-component of angular velocity
        """
        super(AllocatePowerTask, self).__init__()
        linear = Vector3(x=x,y=y,z=z)
        angular = Vector3(x=roll, y=pitch, z=yaw)
        self.twist_power = Twist(linear=linear, angular=angular)  # Twist representing six components of power

    def _on_task_run(self):
        self.publish_desired_twist_power(self.twist_power)


class MoveToPoseLocalTask(MoveToPoseGlobalTask):
    """Move to pose given in local coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw):
        super(MoveToPoseLocalTask, self).__init__(x, y, z, roll, pitch, yaw)

    def _on_task_start(self):
        self.transformed_pose = task_utils.transform('base_link', 'odom', self.desired_pose)

    def _on_task_run(self):
        self.publish_desired_pose_global(self.transformed_pose)
        at_desired_pose_vel = task_utils.stopped_at_pose(self.state.pose.pose, self.transformed_pose, self.state.twist.twist)
        if at_desired_pose_vel:
            self.finish()


class HoldPositionTask(Task):
    """Hold position for a given number of seconds."""

    def __init__(self, hold_time=None):
        """
        Parameters:
            hold_time (double): time to hold in seconds. If None or 0, hold indefinitely
        """
        super(HoldPositionTask, self).__init__()
        self.hold_time = hold_time

    def _on_task_run(self):
        # print(self.initial_state)
        self.publish_desired_pose_global(self.initial_state.pose.pose)
        if self.hold_time:
            if (rospy.get_rostime() - self.start_time) > self.hold_time:
                self.finish()
