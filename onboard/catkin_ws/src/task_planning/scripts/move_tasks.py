from task import Task
from geometry_msgs.msg import Pose, Quaternion, Twist, Point, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import task_utils
import rospy
import smach


class MoveToPoseGlobalTask(smach.State):
    """Move to pose given in global coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw, controls):
        super(MoveToPoseGlobalTask, self).__init__(outcomes=['done', 'continue'])

        self.controls = controls
        self.last_pose = None
        self.coords = [x, y, z, roll, pitch, yaw]

    def execute(self, userdata):
        # Get pose from userdata if supplied
        arg_names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for i in range(len(arg_names)):
            if arg_names[i] in userdata:
                self.coords[i] = userdata[arg_names[i]]

        self.desired_pose = Pose()
        self.desired_pose.position = Point(x=self.coords[0], y=self.coords[1], z=self.coords[2])
        self.desired_pose.orientation = Quaternion(
            *
            quaternion_from_euler(
                self.coords[3],
                self.coords[4],
                self.coords[5]))

        new_pose = self.getPose()
        # Only resend the movement goal if our desired pose has changed
        if self.last_pose is None or not task_utils.at_pose(self.last_pose, new_pose, 0.0001, 0.0001):
            self.last_pose = new_pose
            self.controls.move_to_pose_global(new_pose)

        if task_utils.stopped_at_pose(
            self.controls.get_state().pose.pose,
            new_pose,
            self.controls.get_state().twist.twist):
            self.controls.cancel_movement()
            return 'done'

        return 'continue'

    def getPose(self):
        return self.desired_pose


# FIXME what the local position is will change as we move, so if we're looping through this it'll keep recalculating where it's going
class MoveToPoseLocalTask(MoveToPoseGlobalTask):
    """Move to pose given in local coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw, controls, listener):
        super(MoveToPoseLocalTask, self).__init__(x, y, z, roll, pitch, yaw, controls)
        self.listener = listener

    def getPose(self):
        return task_utils.transform_pose(self.listener, 'base_link', 'odom', self.desired_pose)


class AllocateVelocityLocalTask(Task):
    def __init__(self, x, y, z, roll, pitch, yaw, controls):
        super(AllocateVelocityLocalTask, self).__init__(outcomes=['done'])
        self.controls = controls
        self.coords = [x, y, z, roll, pitch, yaw]
        self.last_twist = None

    def run(self, userdata):
        # Get pose from userdata if supplied
        arg_names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for i in range(len(arg_names)):
            if arg_names[i] in userdata:
                self.coords[i] = userdata[arg_names[i]]

        linear = Vector3(x=self.coords[0], y=self.coords[1], z=self.coords[2])
        angular = Vector3(x=self.coords[3], y=self.coords[4], z=self.coords[5])
        self.desired_twist = Twist(linear=linear, angular=angular)

        new_twist = self.getPose()
        # Only resend the movement goal if our desired pose has changed
        if self.last_pose is None or not task_utils.at_vel(self.last_twist, new_twist, 0.0001, 0.0001):
            self.last_pose = new_twist
            self.controls.move_with_velocity(new_twist)

        return 'done'

    def getTwist(self):
        return self.desired_twist


class AllocateVelocityGlobalTask(AllocateVelocityLocalTask):
    """Allocate specified velocity in a direction"""

    def __init__(self, x, y, z, roll, pitch, yaw, controls, listener):
        """
        Parameters:
            x (float): x-component of linear velocity
            y (float): y-component of linear velocity
            z (float): z-component of linear velocity
            roll (float): roll-component of angular velocity;
            pitch (float): pitch-component of angular velocity
            yaw (float): yaw-component of angular velocity
        """
        super(AllocateVelocityGlobalTask, self).__init__(x, y, z, roll, pitch, yaw, controls)
        self.listener = listener

    def getTwist(self):
        return task_utils.transform_pose(self.listener, 'odom', 'base_link', self.desired_twist)


class HoldPositionTask(Task):
    """Hold position for a given number of seconds."""

    def __init__(self, hold_time=0):
        """
        Parameters:
            hold_time (double): time to hold in seconds. If 0, hold indefinitely
        """
        super(HoldPositionTask, self).__init__()
        self.hold_time = hold_time

    def _on_task_run(self):
        # print(self.initial_state)
        self.publish_desired_pose_global(self.initial_state.pose.pose)
        if self.hold_time and (rospy.get_rostime() - self.start_time) > self.hold_time:
            self.finish()
