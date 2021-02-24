from task import Task
from geometry_msgs.msg import Pose, Quaternion, Twist, Point
from tf.transformations import quaternion_from_euler
import task_utils
import rospy


class MoveToPoseGlobalTask(Task):
    """Move to pose given in global coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw, *args, **kwargs):
        super(MoveToPoseGlobalTask, self).__init__(*args, **kwargs)

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

    def __init__(self, twist_power):
        """
        Parameters:
            twist_power (Twist): values [-1,1] representing relative linear or angular velocity.
                0 values represent axes to stabilize on.
                [1,0,0,0,0,0] is full speed in +x-direction and stabilization on all other axes.
        """
        self.twist_power = twist_power

    def _on_task_run(self):
        self.publish_desired_twist_power(self.twist_power)



class MoveToPoseLocalTask(MoveToPoseGlobalTask):
    """Move to pose given in local coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw, *args, **kwargs):
        super(MoveToPoseLocalTask, self).__init__(x, y, z, roll, pitch, yaw, *args, **kwargs)

    def _on_task_start(self):
        self.transformed_pose = task_utils.transform('base_link', 'odom', self.desired_pose)

    def _on_task_run(self):
        self.publish_desired_pose_global(self.transformed_pose)
        at_desired_pose_vel = task_utils.stopped_at_pose(self.state.pose.pose, self.transformed_pose, self.state.twist.twist)
        if at_desired_pose_vel:
            self.finish()


class HoldPositionTask(Task):
    """Hold position for a given number of seconds."""

    def __init__(self, hold_time=None, *args, **kwargs):
        """
        Parameters:
            hold_time (double): time to hold in seconds. If None or 0, hold indefinitely
        """
        super(HoldPositionTask, self).__init__(*args, **kwargs)
        self.hold_time = hold_time

    def _on_task_run(self):
        # print(self.initial_state)
        self.publish_desired_pose_global(self.initial_state.pose.pose)
        if self.hold_time:
            if (rospy.get_rostime() - self.start_time) > self.hold_time:
                self.finish()
