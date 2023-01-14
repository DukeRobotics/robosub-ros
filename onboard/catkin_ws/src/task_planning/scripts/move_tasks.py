from geometry_msgs.msg import Pose, Quaternion, Twist, Point, Vector3
from tf.transformations import quaternion_from_euler
import task_utils
import smach


class MoveToPoseGlobalTask(smach.State):
    """Move to pose given in global coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw, controls):
        """
        Parameters:
            x (float): x-component of position
            y (float): y-component of position
            z (float): z-component of position
            roll (float): roll-component of orientation
            pitch (float): pitch-component of orientation
            yaw (float): yaw-component of orientation
            controls (interface.ControlsInterface): interface to interact with controls
        """
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

        new_pose = self._get_pose()
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

    def _get_pose(self):
        return self.desired_pose


class MoveToPoseLocalTask(MoveToPoseGlobalTask):
    """Move to pose given in local coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw, controls, listener):
        """
        Parameters:
            x (float): x-component of position
            y (float): y-component of position
            z (float): z-component of position
            roll (float): roll-component of orientation
            pitch (float): pitch-component of orientation
            yaw (float): yaw-component of orientation
            controls (interface.ControlsInterface): interface to interact with controls
            listener (tf.TransformListener): transform listener to go from local to global
        """
        super(MoveToPoseLocalTask, self).__init__(x, y, z, roll, pitch, yaw, controls)
        self.listener = listener
        self.first_pose = True

    def _get_pose(self):
        if self.first_pose:
            self.local_pose = task_utils.transform_pose(self.listener, 'base_link', 'odom', self.desired_pose)
            self.first_pose = False
        return self.local_pose


class AllocateVelocityLocalTask(smach.State):
    """Allocate specified velocity in a direction relative to the robot"""

    def __init__(self, x, y, z, roll, pitch, yaw, controls):
        """
        Parameters:
            x (float): x-component of linear velocity
            y (float): y-component of linear velocity
            z (float): z-component of linear velocity
            roll (float): roll-component of angular velocity
            pitch (float): pitch-component of angular velocity
            yaw (float): yaw-component of angular velocity
            controls (interface.ControlsInterface): interface to interact with controls
        """
        super(AllocateVelocityLocalTask, self).__init__(outcomes=['done'])
        self.controls = controls
        self.coords = [x, y, z, roll, pitch, yaw]
        self.last_twist = None

    def execute(self, userdata):
        # Get pose from userdata if supplied
        arg_names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for i in range(len(arg_names)):
            if arg_names[i] in userdata:
                self.coords[i] = userdata[arg_names[i]]

        linear = Vector3(x=self.coords[0], y=self.coords[1], z=self.coords[2])
        angular = Vector3(x=self.coords[3], y=self.coords[4], z=self.coords[5])
        self.desired_twist = Twist(linear=linear, angular=angular)

        new_twist = self._get_twist()

        # Only resend the movement goal if our desired pose has changed
        if self.last_pose is None or not task_utils.at_vel(self.last_twist, new_twist, 0.0001, 0.0001):
            self.last_pose = new_twist
            self.controls.move_with_velocity(new_twist)

        return 'done'

    def _get_twist(self):
        return self.desired_twist


class AllocateVelocityGlobalTask(AllocateVelocityLocalTask):
    """Allocate specified velocity in a direction relative to the global space"""

    def __init__(self, x, y, z, roll, pitch, yaw, controls, listener):
        """
        Parameters:
            x (float): x-component of linear velocity
            y (float): y-component of linear velocity
            z (float): z-component of linear velocity
            roll (float): roll-component of angular velocity
            pitch (float): pitch-component of angular velocity
            yaw (float): yaw-component of angular velocity
            controls (interface.ControlsInterface): interface to interact with controls
            listener (tf.TransformListener): transform listener to go from global to local
        """
        super(AllocateVelocityGlobalTask, self).__init__(x, y, z, roll, pitch, yaw, controls)
        self.listener = listener
        self.first_twist = True

    def _get_twist(self):
        if self.first_twist:
            self.global_twist = task_utils.transform_pose(self.listener, 'odom', 'base_link', self.desired_twist)
            self.first_twist = False
        return self.global_twist


class HoldPositionTask(smach.State):
    """Hold position at the place the robot is at the first time this runs"""

    def __init__(self, controls):
        """
        Parameters:
            controls (interface.ControlsInterface): interface to interact with controls
        """
        super(HoldPositionTask, self).__init__(outcomes=['done'])
        self.controls = controls
        self.first_pose = True

    def execute(self, userdata):
        if self.first_pose:
            self.hold_pose = self.controls.get_state().pose.pose
            self.controls.move_to_pose_global(self.hold_pose)
            self.first_pose = False

        return 'done'
