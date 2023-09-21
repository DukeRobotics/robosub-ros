from geometry_msgs.msg import Pose, Quaternion, Twist, Point, Vector3
from tf.transformations import quaternion_from_euler
import task_utils
import smach
import rospy


class MoveToPoseGlobalTask(smach.State):
    """Move to pose given in global coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw, controls, input_keys=[]):
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
        super(MoveToPoseGlobalTask, self).__init__(outcomes=['done', 'continue'], input_keys=input_keys)

        self.controls = controls
        self.last_pose = None
        self.desired_pose = Pose()
        self.desired_pose.position = Point(x=x, y=y, z=z)
        self.desired_pose.orientation = Quaternion(
            *
            quaternion_from_euler(
                roll,
                pitch,
                yaw))

    def execute(self, _):
        new_pose = self._get_pose()
        # Only resend the movement goal if our desired pose has changed
        if self.last_pose is None or not task_utils.at_pose(self.last_pose, new_pose, 0.0001, 0.0001):
            self.last_pose = new_pose
            self.controls.move_to_pose_global(new_pose)

    def run(self, userdata):
        print("moving to ", self.desired_pose)
        rate = rospy.Rate(15)
        while not (
            self.state and task_utils.stopped_at_pose(
                self.state.pose.pose,
                self.getPose(),
                self.state.twist.twist)):
            self.publish_desired_pose_global(self.getPose())
            rate.sleep()
        return "done"

    def getPose(self):
        return self.desired_pose


class MoveToUserDataPoseGlobalTask(MoveToPoseGlobalTask):
    """Move to pose passed through userdata given in global coordinates."""

    def __init__(self, controls):
        """
        Parameters:
            controls (interface.ControlsInterface): interface to interact with controls
        """
        super(MoveToUserDataPoseGlobalTask, self).__init__(0, 0, 0, 0, 0, 0, controls, input_keys=['pose'])

    def execute(self, userdata):
        # Get pose from userdata
        self.desired_pose = userdata.pose

        return super(MoveToUserDataPoseGlobalTask, self).execute(userdata)


class MoveToPoseLocalTask(MoveToPoseGlobalTask):
    """Move to pose given in local coordinates."""

    def __init__(self, x, y, z, roll, pitch, yaw, controls, listener, input_keys=[]):
        """
        Move to pose given in local coordinates.

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
        super(MoveToPoseLocalTask, self).__init__(x, y, z, roll, pitch, yaw, controls, input_keys=input_keys)
        self.listener = listener
        self.first_pose = True

    def _get_pose(self):
        if self.first_pose:
            self.local_pose = task_utils.transform_pose(self.listener, 'base_link', 'odom', self.desired_pose)
            self.first_pose = False
        return self.local_pose


class MoveToUserDataPoseLocalTask(MoveToPoseLocalTask):
    """Move to pose passed through userdata given in local coordinates."""

    def __init__(self, controls, listener):
        """
        Move to pose passed through userdata given in local coordinates.

        Parameters:
            controls (interface.ControlsInterface): interface to interact with controls
            listener (tf.TransformListener): transform listener to go from local to global
        """
        super(MoveToUserDataPoseLocalTask, self).__init__(0, 0, 0, 0, 0, 0, controls, listener, input_keys=['pose'])

    def execute(self, userdata):
        # Get pose from userdata
        self.desired_pose = userdata.pose

        return super(MoveToUserDataPoseLocalTask, self).execute(userdata)


class AllocateVelocityLocalTask(smach.State):
    """Allocate specified velocity in a direction relative to the robot"""

    def __init__(self, x, y, z, roll, pitch, yaw, controls, input_keys=[]):
        """
        Allocate specified velocity in a direction relative to the robot

        Parameters:
            x (float): x-component of linear velocity
            y (float): y-component of linear velocity
            z (float): z-component of linear velocity
            roll (float): roll-component of angular velocity
            pitch (float): pitch-component of angular velocity
            yaw (float): yaw-component of angular velocity
            controls (interface.ControlsInterface): interface to interact with controls
        """
        super(AllocateVelocityLocalTask, self).__init__(outcomes=['done'], input_keys=input_keys)
        self.controls = controls
        linear = Vector3(x, y, z)
        angular = Vector3(roll, pitch, yaw)
        self.desired_twist = Twist(linear=linear, angular=angular)
        self.last_twist = None

    def execute(self, _):
        new_twist = self._get_twist()

        # Only resend the movement goal if our desired pose has changed
        if self.last_twist is None or not task_utils.at_vel(self.last_twist, new_twist, 0.0001, 0.0001):
            self.last_twist = new_twist
            self.controls.move_with_velocity(new_twist)

        return 'done'

    def _get_twist(self):
        return self.desired_twist


class AllocateUserDataVelocityLocalTask(AllocateVelocityLocalTask):
    """Allocate specified velocity passed through userdata in a direction relative to the robot"""

    def __init__(self, controls):
        """
        Parameters:
            controls (interface.ControlsInterface): interface to interact with controls
        """
        super(AllocateUserDataVelocityLocalTask, self).__init__(0, 0, 0, 0, 0, 0, controls, input_keys=['twist'])
        self.first_pose = True

    def execute(self, userdata):
        # Get pose from userdata if supplied
        self.desired_twist = userdata.twist

    def run(self, userdata):
        # rospy.loginfo("publishing desired twist...")
        rate = rospy.Rate(15)
        while True:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            self.publish_desired_twist(self.desired_twist)
            rate.sleep()


class AllocateVelocityGlobalTask(AllocateVelocityLocalTask):
    """Allocate specified velocity in a direction relative to the global space"""

    def __init__(self, x, y, z, roll, pitch, yaw, controls, listener, input_keys=[]):
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
        super(AllocateVelocityGlobalTask, self).__init__(x, y, z, roll, pitch, yaw, controls, input_keys=input_keys)
        self.listener = listener
        self.first_twist = True

    def _get_twist(self):
        if self.first_twist:
            self.global_twist = task_utils.transform_pose(self.listener, 'odom', 'base_link', self.desired_twist)
            self.first_twist = False
        return self.global_twist


class AllocateUserDataVelocityGlobalTask(AllocateVelocityGlobalTask):
    """Allocate specified velocity passed through userdata in a direction relative to the global space"""

    def __init__(self, controls):
        """
        Parameters:
            controls (interface.ControlsInterface): interface to interact with controls
        """
        super(AllocateUserDataVelocityGlobalTask, self).__init__(0, 0, 0, 0, 0, 0, controls, input_keys=['twist'])

    def execute(self, userdata):
        # Get pose from userdata if supplied
        self.desired_twist = userdata.twist

        return super(AllocateUserDataVelocityGlobalTask, self).execute(userdata)


class HoldPositionTask(smach.State):
    """Hold position at the place the robot is at the first time this runs"""

    def __init__(self, controls):
        """
        Hold position at the place the robot is at the first time this runs

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
