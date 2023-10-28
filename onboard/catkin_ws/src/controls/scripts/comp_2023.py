#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
import controls_utils
from tf import TransformListener
from tf.transformations import quaternion_from_euler
from custom_msgs.msg import CVObject
import copy


class TaskPlanner:
    PUBLISHING_TOPIC_DESIRED_POSE = 'controls/desired_pose'
    PUBLISHING_TOPIC_DESIRED_TWIST = 'controls/desired_twist'
    PUBLISHING_TOPIC_CURRENT_STATE = '/state'
    PUBLISHING_TOPIC_DESIRED_POWER = 'controls/desired_power'

    def __init__(self):

        rospy.init_node('test_state_publisher')
        self.listener = TransformListener()

        self.sonar_requests = rospy.Publisher("controls/desired_feature", String, queue_size=1)

        self.current_setpoint = [100.0, 100.0, 100.0, 0.0, 0.0, 0.0]

        rospy.Subscriber("/controls/x_pos/setpoint", Float64, self._on_receive_data_x)
        rospy.Subscriber("/controls/y_pos/setpoint", Float64, self._on_receive_data_y)
        rospy.Subscriber("/controls/z_pos/setpoint", Float64, self._on_receive_data_z)
        rospy.Subscriber("/controls/roll_pos/setpoint", Float64, self._on_receive_data_roll)
        rospy.Subscriber("/controls/pitch_pos/setpoint", Float64, self._on_receive_data_pitch)
        rospy.Subscriber("/controls/yaw_pos/setpoint", Float64, self._on_receive_data_yaw)
        rospy.Subscriber("/cv/front/buoy_abydos_serpenscaput", CVObject, self._on_receive_data_cv_serpenscaput)
        rospy.Subscriber("/cv/front/buoy_abydos_taurus", CVObject, self._on_receive_data_cv_taurus)
        rospy.Subscriber("/cv/front/gate_abydos", CVObject, self._on_receive_data_cv_gate)
        rospy.Subscriber("/state", Odometry, self._on_receive_state)
        
        self.MOVE_OFFSET_CONSTANT_LINEAR = [0.2, 0.2, 0.2]
        self.MOVE_OFFSET_CONSTANT_ANGULAR = 0.1

        self.listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(10))

        self._pub_desired_pose = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_POSE, Pose, queue_size=3)
        self._pub_desired_twist = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_TWIST, Twist, queue_size=3)
        self._pub_desired_power = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_POWER, Twist, queue_size=3)

        # These values correspond to the desired global pose of the robot
        self.desired_pose_global = Pose()
        self.desired_pose_global.position.x = 0
        self.desired_pose_global.position.y = 0
        self.desired_pose_global.position.z = 0
        self.desired_pose_global.orientation.x = 0
        self.desired_pose_global.orientation.y = 0
        self.desired_pose_global.orientation.z = 0
        self.desired_pose_global.orientation.w = 1

        # These values correspond to the desired local pose of the robot
        self.desired_pose_local = Pose()
        self.desired_pose_local.position.x = 0
        self.desired_pose_local.position.y = 0
        self.desired_pose_local.position.z = 0
        self.desired_pose_local.orientation.x = 0
        self.desired_pose_local.orientation.y = 0
        self.desired_pose_local.orientation.z = 0
        self.desired_pose_local.orientation.w = 1
        self.recalculate_local_pose()

        self.taurus_time = 0
        self.serpenscaput_time = 0
        self.abydos_time = 0

        self.taurus_pose_transformed = Pose()
        self.serpenscaput_pose_transformed = Pose()
        self.abydos_gate_pose_transformed = Pose()

        self.state = Odometry()

        # These values correspond to the desired local twist for the robot
        # Max linear z speed is ~ -0.26 -- ignore (for different mass)
        self.desired_twist = Twist()
        self.desired_twist.linear.x = 0
        self.desired_twist.linear.y = 0
        self.desired_twist.linear.z = 0
        self.desired_twist.angular.x = 0
        self.desired_twist.angular.y = 0
        self.desired_twist.angular.z = 1

        # These values correspond to the desired power for the robot
        self.desired_power = Twist()
        self.desired_power.linear.x = 0
        self.desired_power.linear.y = 0
        self.desired_power.linear.z = 0
        self.desired_power.angular.x = 0
        self.desired_power.angular.y = 0
        self.desired_power.angular.z = 0

    def recalculate_local_pose(self):
        self.desired_pose_transformed = controls_utils.transform_pose(
            self.listener, "base_link", "odom", self.desired_pose_local)

    def publish_desired_twist(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_twist.publish(self.desired_twist)
            rate.sleep()

    def publish_desired_power(self, time):
        rate = rospy.Rate(15)
        for _ in range(time * 15):
            self._pub_desired_power.publish(self.desired_power)
            rate.sleep()

        self.desired_power.linear.x = 0
        self.desired_power.linear.y = 0
        self.desired_power.linear.z = 0
        self.desired_power.angular.x = 0
        self.desired_power.angular.y = 0
        self.desired_power.angular.z = 0
        self._pub_desired_power.publish(self.desired_power)

    def move_to_local_pos_and_stop(self, x, y, z, roll=0, pitch=0, yaw=0):
        self.desired_pose_local.position.x = x
        self.desired_pose_local.position.y = y
        self.desired_pose_local.position.z = z

        q = quaternion_from_euler(roll, pitch, yaw)
        self.desired_pose_local.orientation.x = q[0]
        self.desired_pose_local.orientation.y = q[1]
        self.desired_pose_local.orientation.z = q[2]
        self.desired_pose_local.orientation.w = q[3]

        self.recalculate_local_pose()
        self._pub_desired_pose.publish(self.desired_pose_transformed)
        rospy.sleep(1)

        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_transformed)

            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                # Ignoring 0 0 0 setpoint
                continue

            # Checking if setpoint is reached
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[0] and \
               abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[1] and \
               abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[2] and \
               abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                break

            rate.sleep()

    def move_to_global_pos_and_stop(self, desired_pose):
        self.desired_pose_global = desired_pose

        self._pub_desired_pose.publish(self.desired_pose_global)
        rospy.sleep(1)

        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_global)

            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                # Ignoring 0 0 0 setpoint
                continue

            # Checking if setpoint is reached
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[0] and \
               abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[1] and \
               abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[2] and \
               abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                break

            rate.sleep()

    # Point the robot forward in the global frame
    def global_face_forward(self):
        temp_state = copy.deepcopy(self.state)

        temp_state.pose.pose.orientation.x = 0
        temp_state.pose.pose.orientation.y = 0
        temp_state.pose.pose.orientation.z = 0
        temp_state.pose.pose.orientation.w = 1

        self.move_to_global_pos_and_stop(temp_state.pose.pose)

    def _on_receive_data_x(self, data):
        self.current_setpoint[0] = data.data

    def _on_receive_data_y(self, data):
        self.current_setpoint[1] = data.data

    def _on_receive_data_z(self, data):
        self.current_setpoint[2] = data.data

    def _on_receive_data_roll(self, data):
        self.current_setpoint[3] = data.data

    def _on_receive_data_pitch(self, data):
        self.current_setpoint[4] = data.data

    def _on_receive_data_yaw(self, data):
        self.current_setpoint[5] = data.data

    def _on_receive_data_cv_serpenscaput(self, data):
        self.serpenscaput_time = data.header.stamp.secs

        desired_pose_serpenscaput = Pose()
        desired_pose_serpenscaput.position.x = data.coords.x
        desired_pose_serpenscaput.position.y = data.coords.y
        desired_pose_serpenscaput.position.z = data.coords.z
        desired_pose_serpenscaput.orientation.x = 0
        desired_pose_serpenscaput.orientation.y = 0
        desired_pose_serpenscaput.orientation.z = 0
        desired_pose_serpenscaput.orientation.w = 1

        self.serpenscaput_pose_transformed = controls_utils.transform_pose(
            self.listener, "base_link", "odom", desired_pose_serpenscaput)

        self.serpenscaput_pose_transformed.orientation.x = 0
        self.serpenscaput_pose_transformed.orientation.y = 0
        self.serpenscaput_pose_transformed.orientation.z = 0
        self.serpenscaput_pose_transformed.orientation.w = 1

    def _on_receive_data_cv_taurus(self, data):
        self.taurus_time = data.header.stamp.secs

        desired_pose_taurus = Pose()
        desired_pose_taurus.position.x = data.coords.x
        desired_pose_taurus.position.y = data.coords.y
        desired_pose_taurus.position.z = data.coords.z
        desired_pose_taurus.orientation.x = 0
        desired_pose_taurus.orientation.y = 0
        desired_pose_taurus.orientation.z = 0
        desired_pose_taurus.orientation.w = 1

        self.taurus_pose_transformed = controls_utils.transform_pose(
            self.listener, "base_link", "odom", desired_pose_taurus)

        self.taurus_pose_transformed.orientation.x = 0
        self.taurus_pose_transformed.orientation.y = 0
        self.taurus_pose_transformed.orientation.z = 0
        self.taurus_pose_transformed.orientation.w = 1

    def _on_receive_data_cv_gate(self, data):
        self.abydos_time = data.header.stamp.secs

        desired_pose_abydos_gate = Pose()
        desired_pose_abydos_gate.position.x = data.coords.x
        desired_pose_abydos_gate.position.y = data.coords.y
        desired_pose_abydos_gate.position.z = data.coords.z
        desired_pose_abydos_gate.orientation.x = 0
        desired_pose_abydos_gate.orientation.y = 0
        desired_pose_abydos_gate.orientation.z = 0
        desired_pose_abydos_gate.orientation.w = 1

        self.abydos_gate_pose_transformed = controls_utils.transform_pose(
            self.listener, "base_link", "odom", desired_pose_abydos_gate)

        self.abydos_gate_pose_transformed.orientation.x = 0
        self.abydos_gate_pose_transformed.orientation.y = 0
        self.abydos_gate_pose_transformed.orientation.z = 0
        self.abydos_gate_pose_transformed.orientation.w = 1

    def _on_receive_state(self, data):
        self.state = data

    def style_task(self, clockwise=True):
        power = -0.5 if clockwise else 0.5

        self.desired_power.linear.x = 0
        self.desired_power.linear.y = 0
        self.desired_power.linear.z = -0.8
        self.desired_power.angular.x = 0
        self.desired_power.angular.y = 0
        self.desired_power.angular.z = power
        self.publish_desired_power(13)

    def gate_task_with_style(self, distance, depth):
        rate = rospy.Rate(15)

        self.move_to_local_pos_and_stop(0, 0, depth)

        temp_state = copy.deepcopy(self.state)

        temp_state.pose.pose.position.x = temp_state.pose.pose.position.x + distance
        temp_state.pose.pose.position.z = temp_state.pose.pose.position.z - 0.5
        temp_state.pose.pose.orientation.x = 0
        temp_state.pose.pose.orientation.y = 0
        temp_state.pose.pose.orientation.z = 0
        temp_state.pose.pose.orientation.w = 1

        self._pub_desired_pose.publish(temp_state.pose.pose)

        rate = rospy.Rate(15)
        rospy.sleep(1)

        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(temp_state.pose.pose)

            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                # Ignoring 0 0 0 setpoint
                continue

            # Checking if setpoint is reached
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[0] and \
               abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[1] and \
               abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                break

            rate.sleep()

        temp_state = copy.deepcopy(self.state)

        self.style_task(True)

        # Self correction to original yaw
        temp_state.pose.pose.orientation.x = 0
        temp_state.pose.pose.orientation.y = 0
        temp_state.pose.pose.orientation.z = 0
        temp_state.pose.pose.orientation.w = 1

        self._pub_desired_pose.publish(temp_state.pose.pose)
        rospy.sleep(1)

        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(temp_state.pose.pose)
            rate.sleep()

            # Ignoring 0 yaw setpoint
            if self.current_setpoint[5] == 0:
                continue

            # Checking if yaw setpoint is reached
            if abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                break

    def cv_gate(self, distance, depth):
        rate = rospy.Rate(15)

        self.move_to_local_pos_and_stop(0, 0, depth)

        after_gate_dead_reckon = copy.deepcopy(self.state)
        after_gate_dead_reckon.pose.pose.position.x = after_gate_dead_reckon.pose.pose.position.x + distance
        after_gate_dead_reckon.pose.pose.orientation.x = 0
        after_gate_dead_reckon.pose.pose.orientation.y = 0
        after_gate_dead_reckon.pose.pose.orientation.z = 0
        after_gate_dead_reckon.pose.pose.orientation.w = 1

        start_time = rospy.Time.now().secs

        while not rospy.is_shutdown():
            time = rospy.Time.now().secs
            if abs(self.abydos_time - time) < 3:
                break

            if abs(time - start_time) > 5:
                while not rospy.is_shutdown():
                    self._pub_desired_pose.publish(after_gate_dead_reckon.pose.pose)
                    if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                        continue
                    if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[0] and \
                       abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[1] and \
                       abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                        break
                    rate.sleep()

                temp_state = copy.deepcopy(self.state)

                self.style_task(True)
                temp_state.pose.pose.orientation.x = 0
                temp_state.pose.pose.orientation.y = 0
                temp_state.pose.pose.orientation.z = 0
                temp_state.pose.pose.orientation.w = 1

                self._pub_desired_pose.publish(temp_state.pose.pose)

                rospy.sleep(1)

                while not rospy.is_shutdown():
                    self._pub_desired_pose.publish(temp_state.pose.pose)
                    rate.sleep()

                    if self.current_setpoint[5] == 0:
                        continue

                    if abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                        break

                return

            if self.abydos_gate_pos_x == 0 or self.abydos_gate_pos_y == 0:
                temp_state = copy.deepcopy(self.state)
                temp_state.pose.pose.position.x = temp_state.pose.pose.position.x + 1
                temp_state.pose.pose.position.z = depth
                temp_state.pose.pose.orientation.x = 0
                temp_state.pose.pose.orientation.y = 0
                temp_state.pose.pose.orientation.z = 0
                temp_state.pose.pose.orientation.w = 1
                self._pub_desired_pose.publish(temp_state.pose.pose)

                rate.sleep()
                continue

        rospy.sleep(1)

        self.abydos_gate_pose_transformed.position.x = self.abydos_gate_pose_transformed.position.x + 1
        self.abydos_gate_pose_transformed.position.z = self.abydos_gate_pose_transformed.position.z - 1
        self._pub_desired_pose.publish(self.abydos_gate_pose_transformed)
        while not rospy.is_shutdown():
            self.abydos_gate_pose_transformed.position.x = self.abydos_gate_pose_transformed.position.x + 1
            self.abydos_gate_pose_transformed.position.z = self.abydos_gate_pose_transformed.position.z - 1
            self._pub_desired_pose.publish(self.abydos_gate_pose_transformed)

            rate.sleep()

            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                continue

            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[0] and \
               abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[1] and \
               abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[2] and \
               abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                break

            if self.abydos_time != 0 and abs(self.abydos_time - rospy.Time.now().secs) > 2:
                break

        temp_state = copy.deepcopy(self.state)

        self.style_task(True)
        temp_state.pose.pose.orientation.x = 0
        temp_state.pose.pose.orientation.y = 0
        temp_state.pose.pose.orientation.z = 0
        temp_state.pose.pose.orientation.w = 1

        self._pub_desired_pose.publish(temp_state.pose.pose)

        rospy.sleep(1)

        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(temp_state.pose.pose)
            rate.sleep()

            if self.current_setpoint[5] == 0:
                continue

            if abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                break

    def buoy_task(self, initial_depth):
        rate = rospy.Rate(15)

        self.move_to_local_pos_and_stop(0, 0, initial_depth)

        rospy.sleep(0.5)

        start_time = rospy.Time.now().secs
        while not rospy.is_shutdown():
            time = rospy.Time.now().secs

            # Checking for recent Taurus detection
            if abs(self.taurus_time - time) < 3:
                rospy.sleep(1)
                break

            if abs(time - start_time) > 20:
                # Timed out
                return

            # If Taurus not detected, do a circular scan to find Taurus
            self.desired_power.linear.x = 0
            self.desired_power.linear.y = 0
            self.desired_power.linear.z = -0.45
            self.desired_power.angular.x = 0
            self.desired_power.angular.y = 0
            self.desired_power.angular.z = -0.15
            self.publish_desired_power(0.5)

            rate.sleep()

        self.sonar_requests.publish("buoy_abydos_taurus")
        temp_state = copy.deepcopy(self.state)

        self.taurus_pose_transformed.position.x = self.taurus_pose_transformed.position.x + 0.25
        self.taurus_pose_transformed.position.y = self.taurus_pose_transformed.position.y
        self.taurus_pose_transformed.orientation = temp_state.pose.pose.orientation

        self._pub_desired_pose.publish(self.taurus_pose_transformed)

        while not rospy.is_shutdown():
            self.taurus_pose_transformed.position.x = self.taurus_pose_transformed.position.x + 0.25
            self.taurus_pose_transformed.position.y = self.taurus_pose_transformed.position.y
            self.taurus_pose_transformed.orientation = temp_state.pose.pose.orientation
            self._pub_desired_pose.publish(self.taurus_pose_transformed)

            rate.sleep()

            # Ignoring 0 0 0 setpoint
            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                continue

            # Checking whether setpoint is reached
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[0] and \
               abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[1] and \
               abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[2] and \
               abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                break

            if self.taurus_time != 0 and abs(self.taurus_time - rospy.Time.now().secs) > 3:
                # Taurus not detected for 3 seconds, count as a hit
                break

        rospy.sleep(1)

        temp_state = copy.deepcopy(self.state)

        temp_state.pose.pose.position.x = temp_state.pose.pose.position.x - 2
        temp_state.pose.pose.position.y = temp_state.pose.pose.position.y + 0.5

        temp_state.pose.pose.orientation.x = 0
        temp_state.pose.pose.orientation.y = 0
        temp_state.pose.pose.orientation.z = 0
        temp_state.pose.pose.orientation.w = 1

        # Moving back
        self.move_to_global_pos_and_stop(temp_state.pose.pose)

        rospy.sleep(1)

        start_time = rospy.Time.now().secs
        while not rospy.is_shutdown():
            time = rospy.Time.now().secs

            # Checking for recent Serpens Caput detection
            if abs(self.serpenscaput_time - time) < 3:
                break

            if abs(time - start_time) > 20:
                # Timed out, dead reckon 2nd buoy
                temp_state.pose.pose.position.x = temp_state.pose.pose.position.x + 2.25
                temp_state.pose.pose.position.y = temp_state.pose.pose.position.y + 0.2
                temp_state.pose.pose.position.z = temp_state.pose.pose.position.z + 0.9

                self.move_to_global_pos_and_stop(temp_state.pose.pose)

                return

            # If Serpens Caput not detected, do a circular scan to find Serpens Caput
            self.desired_power.linear.x = 0
            self.desired_power.linear.y = 0
            self.desired_power.linear.z = -0.45
            self.desired_power.angular.x = 0
            self.desired_power.angular.y = 0
            self.desired_power.angular.z = 0.15
            self.publish_desired_power(0.5)

            rate.sleep()

        self.sonar_requests.publish("buoy_abydos_serpenscaput")
        temp_state = copy.deepcopy(self.state)

        self.serpenscaput_pose_transformed.position.x = self.serpenscaput_pose_transformed.position.x + 0.25
        self.serpenscaput_pose_transformed.orientation = temp_state.pose.pose.orientation

        self._pub_desired_pose.publish(self.serpenscaput_pose_transformed)

        while not rospy.is_shutdown():
            self.serpenscaput_pose_transformed.position.x = self.serpenscaput_pose_transformed.position.x + 0.25
            self.serpenscaput_pose_transformed.orientation = temp_state.pose.pose.orientation
            self._pub_desired_pose.publish(self.serpenscaput_pose_transformed)

            rate.sleep()

            # Ignoring 0 0 0 setpoint
            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                continue

            # Checking whether setpoint is reached
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[0] and \
               abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[1] and \
               abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT_LINEAR[2] and \
               abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                break

            if self.serpenscaput_time != 0 and abs(self.serpenscaput_time - rospy.Time.now().secs) > 3:
                # Serpens Caput not detected for 3 seconds, count as a hit
                break

    def octagon_task(self):
        self.move_to_local_pos_and_stop(-1, -2, 0.5)

        octagon = Pose()

        octagon.position.x = 33.8
        octagon.position.y = -11.8
        octagon.position.z = -1.5

        octagon.orientation.x = 0
        octagon.orientation.y = 0
        octagon.orientation.z = 0
        octagon.orientation.w = 1

        self.move_to_global_pos_and_stop(octagon)

        octagon.position.z = 0

        self.move_to_global_pos_and_stop(octagon)


def main():
    task_planner = TaskPlanner()

    # task_planner.buoy_task(-0.75)

    # task_planner.move_to_local_pos_and_stop(0, 0, -1)
    # task_planner.move_to_local_pos_and_stop(1, 0, 0)

    # Competition code below
    # rospy.sleep(10)

    task_planner.gate_task_with_style(3, -0.5)
    # rospy.sleep(1)
    # task_planner.buoy_task(0)
    # task_planner.octagon_task()


if __name__ == '__main__':
    main()
