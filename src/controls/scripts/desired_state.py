#!/usr/bin/env python

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from std_msgs.msg import Float64, Boolean
from drc_math import RpToTrans, Adjoint

from geometry_msgs.msg import Pose, Twist
# from nav_msgs.msg import Odometry

class DesiredStateHandler():

    POSE_TOPIC_X = '/controls/state/pose/x'
    POSE_TOPIC_Y = '/controls/state/pose/y'
    POSE_TOPIC_Z = '/controls/state/pose/z'
    POSE_TOPIC_ROLL = '/controls/state/pose/roll'
    POSE_TOPIC_PITCH = '/controls/state/pose/pitch'
    POSE_TOPIC_YAW = '/controls/state/pose/yaw'

    DESIRED_LOCAL_TWIST_TOPIC = 'controls/desired_twist_local'
    DESIRED_GLOBAL_TWIST_TOPIC = 'controls/desired_twist_global'
    DESIRED_POSE_TOPIC = 'controls/desired_pose_global'

    PUBLISHING_TOPIC_X_POS = 'controls/x_pos/setpoint'
    PUBLISHING_TOPIC_Y_POS = 'controls/y_pos/setpoint'
    PUBLISHING_TOPIC_Z_POS = 'controls/z_pos/setpoint'
    PUBLISHING_TOPIC_ROLL_POS = 'controls/roll_pos/setpoint'
    PUBLISHING_TOPIC_PITCH_POS = 'controls/pitch_pos/setpoint'
    PUBLISHING_TOPIC_YAW_POS = 'controls/yaw_pos/setpoint'

    PUBLISHING_TOPIC_X_VEL = 'controls/x_vel/setpoint'
    PUBLISHING_TOPIC_Y_VEL = 'controls/y_vel/setpoint'
    PUBLISHING_TOPIC_Z_VEL = 'controls/z_vel/setpoint'
    PUBLISHING_TOPIC_ROLL_VEL = 'controls/roll_vel/setpoint'
    PUBLISHING_TOPIC_PITCH_VEL = 'controls/pitch_vel/setpoint'
    PUBLISHING_TOPIC_YAW_VEL = 'controls/yaw_vel/setpoint'

    PUBLISHING_TOPIC_ENABLE_X_POS = 'controls/enable/x_pos'
    PUBLISHING_TOPIC_ENABLE_Y_POS = 'controls/enable/y_pos'
    PUBLISHING_TOPIC_ENABLE_Z_POS = 'controls/enable/z_pos'
    PUBLISHING_TOPIC_ENABLE_ROLL_POS = 'controls/enable/roll_pos'
    PUBLISHING_TOPIC_ENABLE_PITCH_POS = 'controls/enable/pitch_pos'
    PUBLISHING_TOPIC_ENABLE_YAW_POS = 'controls/enable/yaw_pos'

    PUBLISHING_TOPIC_ENABLE_X_VEL = 'controls/enable/x_vel'
    PUBLISHING_TOPIC_ENABLE_Y_VEL = 'controls/enable/y_vel'
    PUBLISHING_TOPIC_ENABLE_Z_VEL = 'controls/enable/z_vel'
    PUBLISHING_TOPIC_ENABLE_ROLL_VEL = 'controls/enable/roll_vel'
    PUBLISHING_TOPIC_ENABLE_PITCH_VEL = 'controls/enable/pitch_vel'
    PUBLISHING_TOPIC_ENABLE_YAW_VEL = 'controls/enable/yaw_vel'


    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0

    adjoint = np.zeros(6, 6)

    pose = None
    local_twist = None
    global_twist = None

    def __init__(self):
        rospy.Subscriber(self.POSE_TOPIC_X, Float64, self.receive_x)
        rospy.Subscriber(self.POSE_TOPIC_Y, Float64, self.receive_y)
        rospy.Subscriber(self.POSE_TOPIC_Z, Float64, self.receive_z)
        rospy.Subscriber(self.POSE_TOPIC_ROLL, Float64, self.receive_roll)
        rospy.Subscriber(self.POSE_TOPIC_PITCH, Float64, self.receive_pitch)
        rospy.Subscriber(self.POSE_TOPIC_YAW, Float64, self.receive_yaw)

        #PID Position publishers
        self._pub_x_pos = rospy.Publisher(self.PUBLISHING_TOPIC_X_POS, Float64, queue_size=3)
        self._pub_y_pos = rospy.Publisher(self.PUBLISHING_TOPIC_Y_POS, Float64, queue_size=3)
        self._pub_z_pos = rospy.Publisher(self.PUBLISHING_TOPIC_Z_POS, Float64, queue_size=3)
        self._pub_roll_pos = rospy.Publisher(self.PUBLISHING_TOPIC_ROLL_POS, Float64, queue_size=3)
        self._pub_pitch_pos = rospy.Publisher(self.PUBLISHING_TOPIC_PITCH_POS, Float64, queue_size=3)
        self._pub_yaw_pos = rospy.Publisher(self.PUBLISHING_TOPIC_YAW_POS, Float64, queue_size=3)
        #PID Velocity publishers
        self._pub_x_vel = rospy.Publisher(self.PUBLISHING_TOPIC_X_VEL, Float64, queue_size=3)
        self._pub_y_vel = rospy.Publisher(self.PUBLISHING_TOPIC_Y_VEL, Float64, queue_size=3)
        self._pub_z_vel = rospy.Publisher(self.PUBLISHING_TOPIC_Z_VEL, Float64, queue_size=3)
        self._pub_roll_vel = rospy.Publisher(self.PUBLISHING_TOPIC_ROLL_VEL, Float64, queue_size=3)
        self._pub_pitch_vel = rospy.Publisher(self.PUBLISHING_TOPIC_PITCH_VEL, Float64, queue_size=3)
        self._pub_yaw_vel = rospy.Publisher(self.PUBLISHING_TOPIC_YAW_VEL, Float64, queue_size=3)

        self._pub_x_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_X_POS, Boolean, queue_size=3)
        self._pub_y_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_Y_POS, Boolean, queue_size=3)
        self._pub_z_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_Z_POS, Boolean, queue_size=3)
        self._pub_roll_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_ROLL_POS, Boolean, queue_size=3)
        self._pub_pitch_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_PITCH_POS, Boolean, queue_size=3)
        self._pub_yaw_pos_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_YAW_POS, Boolean, queue_size=3)

        self._pub_x_vel_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_X_VEL, Boolean, queue_size=3)
        self._pub_y_vel_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_Y_VEL, Boolean, queue_size=3)
        self._pub_z_vel_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_Z_VEL, Boolean, queue_size=3)
        self._pub_roll_vel_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_ROLL_VEL, Boolean, queue_size=3)
        self._pub_pitch_vel_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_PITCH_VEL, Boolean, queue_size=3)
        self._pub_yaw_vel_enable = rospy.Publisher(self.PUBLISHING_TOPIC_ENABLE_YAW_VEL, Boolean, queue_size=3)


        rospy.Subscriber(self.DESIRED_POSE_TOPIC, Pose, self.receive_pose)
        rospy.Subscriber(self.DESIRED_LOCAL_TWIST_TOPIC, Twist, self.receive_local_twist)
        rospy.Subscriber(self.DESIRED_GLOBAL_TWIST_TOPIC, Twist, self.receive_global_twist)

    def receive_x(self, x):
        self.x = x
        self.update_transformation()

    def receive_y(self, y):
        self.y = y
        self.update_transformation()

    def receive_z(self, z):
        self.z = z
        self.update_transformation()

    def receive_roll(self, roll):
        self.roll = roll
        self.update_transformation()

    def receive_pitch(self, pitch):
        self.pitch = pitch
        self.update_transformation()

    def receive_yaw(self, yaw):
        self.yaw = yaw
        self.update_transformation()

    def receive_pose(self, pose):
        self.pose = pose

    def receive_local_twist(self, twist):
        self.local_twist = twist

    def receive_global_twist(self, twist):
        self.global_twist = twist

    def update_transformation(self):
        # Create rotation matrix from local frame to global frame
        r = euler_matrix(-self.roll, -self.pitch, -self.yaw)
        
        # Create translation vector from local frame to global frame
        p = np.array([-self.x, -self.y, -self.z])

        # Create transformation matrix
        t = RpToTrans(r, p)

        # Create adjoint matrix for twist transformation
        self.adjoint = Adjoint(t)

    def run(self):

        rospy.init_node('desired_state')
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.pose and not self.local_twist and not self.global_twist:
                # TODO: If it's been too long, stop PID and warn (timeout)

                continue
            if (self.pose and self.local_twist) or (self.pose and self.global_twist) or (self.local_twist and self.global_twist):
                # Make this if statement prettier
                # TODO: More than one seen in one update cycle, so warn and continue
                continue

            # Now we have either pose XOR local_twist XOR global_twist

            if self.pose:
                self._pub_x_pos_enable.publish(True)
                self._pub_y_pos_enable.publish(True)
                self._pub_z_pos_enable.publish(True)
                self._pub_roll_pos_enable.publish(True)
                self._pub_pitch_pos_enable.publish(True)
                self._pub_yaw_pos_enable.publish(True)

                self._pub_x_vel_enable.publish(False)
                self._pub_y_vel_enable.publish(False)
                self._pub_z_vel_enable.publish(False)
                self._pub_roll_vel_enable.publish(False)
                self._pub_pitch_vel_enable.publish(False)
                self._pub_yaw_vel_enable.publish(False)


                x = self.pose.position.x
                y = self.pose.position.y
                z = self.pose.position.z

                orientation = self.pose.orientation
                roll, pitch, yaw = euler_from_quaternion([orientation.x,
                                                          orientation.y,
                                                          orientation.z,
                                                          orientation.w])

                self._pub_x_pos.publish(x)
                self._pub_y_pos.publish(y)
                self._pub_z_pos.publish(z)
                self._pub_roll_pos.publish(roll)
                self._pub_pitch_pos.publish(pitch)
                self._pub_yaw_pos.publish(yaw)

                self.pose = None

            elif self.local_twist:
                self._pub_x_pos_enable.publish(False)
                self._pub_y_pos_enable.publish(False)
                self._pub_z_pos_enable.publish(False)
                self._pub_roll_pos_enable.publish(False)
                self._pub_pitch_pos_enable.publish(False)
                self._pub_yaw_pos_enable.publish(False)

                self._pub_x_vel_enable.publish(True)
                self._pub_y_vel_enable.publish(True)
                self._pub_z_vel_enable.publish(True)
                self._pub_roll_vel_enable.publish(True)
                self._pub_pitch_vel_enable.publish(True)
                self._pub_yaw_vel_enable.publish(True)

                x_vel = self.local_twist.linear.x
                y_vel = self.local_twist.linear.y
                z_vel = self.local_twist.linear.z
                roll_vel = self.local_twist.angular.x
                pitch_vel = self.local_twist.angular.y
                yaw_vel = self.local_twist.angular.z

                self._pub_x_vel.publish(x_vel)
                self._pub_y_vel.publish(y_vel)
                self._pub_z_vel.publish(z_vel)
                self._pub_roll_vel.publish(roll_vel)
                self._pub_pitch_vel.publish(pitch_vel)
                self._pub_yaw_vel.publish(yaw_vel)

                self.local_twist = None

            elif self.global_twist:
                self._pub_x_pos_enable.publish(False)
                self._pub_y_pos_enable.publish(False)
                self._pub_z_pos_enable.publish(False)
                self._pub_roll_pos_enable.publish(False)
                self._pub_pitch_pos_enable.publish(False)
                self._pub_yaw_pos_enable.publish(False)

                self._pub_x_vel_enable.publish(True)
                self._pub_y_vel_enable.publish(True)
                self._pub_z_vel_enable.publish(True)
                self._pub_roll_vel_enable.publish(True)
                self._pub_pitch_vel_enable.publish(True)
                self._pub_yaw_vel_enable.publish(True)

                # Perform necessary transformations from global to local frame then publish
                g = np.array([self.global_twist.linear.x,
                              self.global_twist.linear.y,
                              self.global_twist.linear.z,
                              self.global_twist.angular.x,
                              self.global_twist.angular.y,
                              self.global_twist.angular.z])
                local = np.matmul(self.adjoint, g)

                self._pub_x_vel.publish(local[0])
                self._pub_y_vel.publish(local[1])
                self._pub_z_vel.publish(local[2])
                self._pub_roll_vel.publish(local[3])
                self._pub_pitch_vel.publish(local[4])
                self._pub_yaw_vel.publish(local[5])

                self.global_twist = None

            rate.sleep()


def main():
    DesiredStateHandler().run()

if __name__ == '__main__':
    main()
