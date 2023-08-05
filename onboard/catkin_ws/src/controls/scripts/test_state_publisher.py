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


class TestStatePublisher:
    PUBLISHING_TOPIC_DESIRED_POSE = 'controls/desired_pose'
    PUBLISHING_TOPIC_DESIRED_TWIST = 'controls/desired_twist'
    PUBLISHING_TOPIC_CURRENT_STATE = '/state'
    PUBLISHING_TOPIC_DESIRED_POWER = 'controls/desired_power'

    def recalculate_local_pose(self):
        self.desired_pose_transformed = controls_utils.transform_pose(
            self.listener, "base_link", "odom", self.desired_pose_local)

    def __init__(self):
        
        rospy.init_node('test_state_publisher')
        self.listener = TransformListener()

        self.sonar_requests = rospy.Publisher("controls/desired_feature", String)
        
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

        self.current_setpoint = [100.0, 100.0, 100.0, 0.0, 0.0, 0.0] # x,y,z,r,p,y
        self.MOVE_OFFSET_CONSTANT = [0.2, 0.2, 0.2]  # x,y,z
        self.MOVE_OFFSET_CONSTANT_ANGULAR = 0.1

        self.listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(10))

        self._pub_desired_pose = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_POSE, Pose, queue_size=3)
        self._pub_desired_twist = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_TWIST, Twist, queue_size=3)
        self._pub_desired_power = rospy.Publisher(self.PUBLISHING_TOPIC_DESIRED_POWER, Twist, queue_size=3)
        self._pub_current_state = rospy.Publisher(self.PUBLISHING_TOPIC_CURRENT_STATE, Odometry, queue_size=3)

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

        self.current_pos_x = 0
        self.current_pos_y = 0
        self.current_pos_z = 0

        self.taurus_pos_x = 0
        self.taurus_pos_y = 0
        self.taurus_pos_z = 0
        self.taurus_yaw = 0
        
        self.taurus_time = 0
        
        self.serpenscaput_pos_x = 0
        self.serpenscaput_pos_y = 0
        self.serpenscaput_pos_z = 0
        self.serpenscaput_yaw = 0
        
        self.serpenscaput_time = 0

        self.abydos_gate_pos_x = 0
        self.abydos_gate_pos_y = 0
        self.abydos_gate_pos_z = 0
        self.abydos_gate_yaw = 0

        self.state = Odometry()

        self.initial_yaw = 0

        # These values correspond to the desired local twist for the robot
        # Max linear z speed is ~ -0.26 -- ignore (for different mass)
        self.desired_twist = Twist()
        self.desired_twist.linear.x = 0
        self.desired_twist.linear.y = 0
        self.desired_twist.linear.z = 0
        self.desired_twist.angular.x = 0
        self.desired_twist.angular.y = 0
        self.desired_twist.angular.z = 1

        # These values correspond to the desired twist for the robot
        self.desired_power = Twist()
        self.desired_power.linear.x = 0
        self.desired_power.linear.y = 0
        self.desired_power.linear.z = 0
        self.desired_power.angular.x = 0
        self.desired_power.angular.y = 0
        self.desired_power.angular.z = 0

        self.current_state = Odometry()
        self.current_state.pose.pose.position.x = 0
        self.current_state.pose.pose.position.y = 0
        self.current_state.pose.pose.position.z = 0
        self.current_state.pose.pose.orientation.x = 0
        self.current_state.pose.pose.orientation.y = 0
        self.current_state.pose.pose.orientation.z = 0
        self.current_state.pose.pose.orientation.w = 1

        self.current_state.twist.twist.linear.x = 0
        self.current_state.twist.twist.linear.y = 0
        self.current_state.twist.twist.linear.z = 0
        self.current_state.twist.twist.angular.x = 0
        self.current_state.twist.twist.angular.y = 0
        self.current_state.twist.twist.angular.z = 0

        self.current_state.header.frame_id = 'odom'
        self.current_state.header.stamp = rospy.Time()

        self.taurus_pose_transformed = Pose()
        self.serpenscaput_pose_transformed = Pose()

    def publish_desired_pose_global(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_global)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()

    def publish_desired_pose_local(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            rate.sleep()

    def publish_desired_twist(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_twist.publish(self.desired_twist)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()

    def publish_desired_power(self, time):
        rate = rospy.Rate(15)
        timer = 0
        while not rospy.is_shutdown():
            if (timer >= time * 15):
                break
            
            timer += 1
            self._pub_desired_power.publish(self.desired_power)
            rate.sleep()

        self.desired_power.linear.x = 0
        self.desired_power.linear.y = 0
        self.desired_power.linear.z = 0
        self.desired_power.angular.x = 0
        self.desired_power.angular.y = 0
        self.desired_power.angular.z = 0
        self._pub_desired_power.publish(self.desired_power)
        # self._pub_current_state.publish(self.current_state)
        # rospy.sleep(time)
        # return

    def move_to_pos_and_stop(self, x, y, z, roll=0, pitch=0, yaw=0):

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
            print(self.current_setpoint)

            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                print("Ignoring 0 0 0 setpoint")
                continue
            
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and \
               abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and \
               abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2] and \
               abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                print("reached setpoint")
                break

            rate.sleep()
            
    def move_to_global_pos_and_stop(self, desired_pose):

        self.desired_pose_global = desired_pose

        self._pub_desired_pose.publish(self.desired_pose_global)
        rospy.sleep(1)

        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_global)
            print(self.current_setpoint)

            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                print("Ignoring 0 0 0 setpoint")
                continue
            
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and \
               abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and \
               abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2] and \
               abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                print("reached setpoint")
                break

            rate.sleep()

    def move_forward_with_yaw(self, x):
        self.desired_pose_local.position.x = self.state + x
        self.desired_pose_local.position.y = self.state
        self.desired_pose_local.position.z = self.state

        q = quaternion_from_euler(roll, pitch, yaw)
        self.desired_pose_local.orientation.x = q[0]
        self.desired_pose_local.orientation.y = q[1]
        self.desired_pose_local.orientation.z = q[2]
        self.desired_pose_local.orientation.w = q[3]

        self.recalculate_local_pose()

        # q_initial = quaternion_from_euler(0, 0, self.initial_yaw)
        # self.desired_pose_transformed.orientation.x = q_initial[0]
        # self.desired_pose_transformed.orientation.y = q_initial[1]
        # self.desired_pose_transformed.orientation.z = q_initial[2]
        # self.desired_pose_transformed.orientation.w = q_initial[3]

        self.desired_pose_transformed.orientation.x = 0
        self.desired_pose_transformed.orientation.y = 0
        self.desired_pose_transformed.orientation.z = 0
        self.desired_pose_transformed.orientation.w = 1

        self._pub_desired_pose.publish(self.desired_pose_transformed)
        rospy.sleep(1)

        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            print(self.current_setpoint)

            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                print("Ignoring 0 0 0 setpoint")
                continue
            
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and \
               abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and \
               abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2] and \
               abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                print("reached setpoint")
                break

            rate.sleep()

    def move_to_yaw_and_stop(self, x, y, z, w):
        # self.desired_pose_local.position.x = 0
        # self.desired_pose_local.position.y = 0
        # self.desired_pose_local.position.z = z
        self.desired_pose_local.orientation.x = x
        self.desired_pose_local.orientation.y = y
        self.desired_pose_local.orientation.z = z
        self.desired_pose_local.orientation.w = w

        self.recalculate_local_pose()

        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            if abs(self.current_state[3]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR and \
               abs(self.current_state[4]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR and \
               abs(self.current_state[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                break
            rate.sleep()

    # Point the robot forward in the global frame
    def global_face_forward(self):
        # Set desired local pose to current postion
        self.desired_pose_local.position.x = 0
        self.desired_pose_local.position.y = 0
        self.desired_pose_local.position.z = 0
        self.desired_pose_local.orientation.x = 0
        self.desired_pose_local.orientation.y = 0
        self.desired_pose_local.orientation.z = 0
        self.desired_pose_local.orientation.w = 1

        # Convert to global pose
        self.recalculate_local_pose()

        # Set global pose to point forward relative to start orientation
        self.desired_pose_transformed.orientation.x = self.global_start_pose.orientation.x
        self.desired_pose_transformed.orientation.y = self.global_start_pose.orientation.y
        self.desired_pose_transformed.orientation.z = self.global_start_pose.orientation.z
        self.desired_pose_transformed.orientation.w = self.global_start_pose.orientation.w

        rate = rospy.Rate(15)

        # DELAY BASED STOPPING PROTOCOL
        # tot = 0
        # duration = 60_000
        # while tot < duration and not rospy.is_shutdown():
        #     self._pub_desired_pose.publish(self.desired_pose_transformed)
        #     tot += 15
        #     rate.sleep()

        # SET POINT BASED STOPPING PROTOCOL
        
        delay = 0
        while not rospy.is_shutdown():
            delay += 1
            self._pub_desired_pose.publish(self.desired_pose_transformed)

            if delay > 30:
                if abs(self.current_yaw) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                    break
            rate.sleep()

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

        if self.initial_yaw == 0:
            self.initial_yaw = data.data

    def _on_receive_data_cv_serpenscaput(self, data):
        self.serpenscaput_pos_x = data.coords.x
        self.serpenscaput_pos_y = data.coords.y
        self.serpenscaput_pos_z = data.coords.z
        self.serpenscaput_yaw = data.yaw

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
        self.taurus_pos_x = data.coords.x
        self.taurus_pos_y = data.coords.y
        self.taurus_pos_z = data.coords.z
        self.taurus_yaw = data.yaw
        
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
        # self.taurus_pose_transformed = copy.deepcopy(self.state.pose.pose)
        # self.taurus_pose_transformed.position.x += desired_pose_taurus.position.x
        # self.taurus_pose_transformed.position.y += desired_pose_taurus.position.y
        # self.taurus_pose_transformed.position.z += desired_pose_taurus.position.z

    def _on_receive_data_cv_gate(self, data):
        self.abydos_gate_pos_x = data.coords.x
        self.abydos_gate_pos_y = data.coords.y
        self.abydos_gate_pos_z = data.coords.z
        self.abydos_gate_yaw = data.yaw        

    def _on_receive_state(self, data):
        self.state = data

    def yaw_720(self, clockwise=True):
        power = -0.5 if clockwise else 0.5

        self.desired_power.linear.x = 0
        self.desired_power.linear.y = 0
        self.desired_power.linear.z = -0.8
        self.desired_power.angular.x = 0
        self.desired_power.angular.y = 0
        self.desired_power.angular.z = power
        self.publish_desired_power(13)

        print("Stopped yaw 720")

    def yaw_task(self):
        print("turning")
        self.yaw_720(True)

        print("moving back to temp state")

    def style_and_return(self):
        print(f"Starting test_state_publisher.py")
        self.move_to_pos_and_stop(0, 0, -0.75) # submerge
        
        print("Submerged")
        temp_state = copy.deepcopy(self.state)
        print(temp_state)
        
        self.move_to_pos_and_stop(2, 0, 0) # forward
        print("Starting yaw")
        
        self.yaw_task()

        print("Finished yaw, move back to previous global state")
        self.update_desired_pos_global(temp_state)

    def update_desired_pos_local(self, pos_x, pos_y, pos_z,
                                 roll=0, pitch=0, yaw=0):
        
        self.desired_pose_local.position.x = pos_x
        self.desired_pose_local.position.y = pos_y
        self.desired_pose_local.position.z = pos_z

        q = quaternion_from_euler(roll, pitch, yaw)
        self.desired_pose_local.orientation.x = q[0]
        self.desired_pose_local.orientation.y = q[1]
        self.desired_pose_local.orientation.z = q[2]
        self.desired_pose_local.orientation.w = q[3]

        self.recalculate_local_pose()
        self._pub_desired_pose.publish(self.desired_pose_transformed)

    def update_desired_pos_global(self, state):
        
        self.desired_pose_global.position.x = state.pose.pose.position.x
        self.desired_pose_global.position.y = state.pose.pose.position.y
        self.desired_pose_global.position.z = state.pose.pose.position.z

        self.desired_pose_global.orientation.x = state.pose.pose.orientation.x
        self.desired_pose_global.orientation.y = state.pose.pose.orientation.y
        self.desired_pose_global.orientation.z = state.pose.pose.orientation.z
        self.desired_pose_global.orientation.w = state.pose.pose.orientation.w

        self._pub_desired_pose.publish(self.desired_pose_global)

    def dead_reckon_gate_with_style(self, distance, depth):
        rate = rospy.Rate(15)

        self.move_to_pos_and_stop(0, 0, depth)
        print("Submerged")

        self.move_to_pos_and_stop(distance, 0, -0.5)
        print("Moved forward")

        # temp_state = copy.deepcopy(self.state)
        # print(temp_state)

        self.yaw_720(True)
        print("Finished yaw, move back to previous global state")

        # temp_state.pose.pose.position.x = 0
        # temp_state.pose.pose.position.y = 0
        # temp_state.pose.pose.position.z = 0
        # temp_state.pose.pose.orientation.x = 0
        # temp_state.pose.pose.orientation.y = 0
        # temp_state.pose.pose.orientation.z = 0
        # temp_state.pose.pose.orientation.w = 1

        # self.update_desired_pos_global(temp_state)

        # rospy.sleep(1)
        
        # while not rospy.is_shutdown():
        #     self._pub_desired_pose.publish(self.desired_pose_global)
        #     print(f"Yaw setpoint: {self.current_setpoint[5]}")
        #     rate.sleep()

        #     if self.current_setpoint[5] == 0:
        #         continue

        #     if abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
        #         break

        # print("Finished yawing")

        self.move_to_pos_and_stop(0, 0, -1)
        print("Finished submerging again")

        # self.move_to_pos_and_stop(1.5, 0, 0)
        print("Finished task")

    def dead_reckon_gate_with_style_with_yaw_correction(self, distance, depth):
        rate = rospy.Rate(15)

        self.move_to_pos_and_stop(0, 0, depth)
        print("Submerged")

        temp_state = copy.deepcopy(self.state)

        temp_state.pose.pose.position.x = temp_state.pose.pose.position.x + distance
        temp_state.pose.pose.position.z = temp_state.pose.pose.position.z - 0.5

        # q = quaternion_from_euler(0, 0, self.initial_yaw)
        # temp_state.pose.pose.orientation.x = q[0]
        # temp_state.pose.pose.orientation.y = q[1]
        # temp_state.pose.pose.orientation.z = q[2]
        # temp_state.pose.pose.orientation.w = q[3]

        temp_state.pose.pose.orientation.x = 0
        temp_state.pose.pose.orientation.y = 0
        temp_state.pose.pose.orientation.z = 0
        temp_state.pose.pose.orientation.w = 1
        
        self.move_to_global_pos_and_stop(temp_state.pose.pose)

        # self.update_desired_pos_global(temp_state)

        # rate = rospy.Rate(15)
        # rospy.sleep(1)

        # while not rospy.is_shutdown():
        #     print(self.current_setpoint)
        #     self._pub_desired_pose.publish(temp_state.pose.pose)

        #     if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
        #         print("Ignoring 0 0 0 setpoint")
        #         continue
            
        #     if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and \
        #     abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and \
        #     abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2] and \
        #     abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
        #         print("reached setpoint")
        #         break

        #     rate.sleep()

        print("Finished moving forward with yaw")

        temp_state = copy.deepcopy(self.state)
        print(temp_state)

        self.yaw_720(True)
        print("Finished yaw, move back to previous global state")

        # q = quaternion_from_euler(0, 0, self.initial_yaw)
        # temp_state.pose.pose.orientation.x = q[0]
        # temp_state.pose.pose.orientation.y = q[1]
        # temp_state.pose.pose.orientation.z = q[2]
        # temp_state.pose.pose.orientation.w = q[3]

        # temp_state.pose.pose.position.x = 0
        # temp_state.pose.pose.position.y = 0
        # temp_state.pose.pose.position.z = 0
        temp_state.pose.pose.orientation.x = 0
        temp_state.pose.pose.orientation.y = 0
        temp_state.pose.pose.orientation.z = 0
        temp_state.pose.pose.orientation.w = 1
        
        
        self.move_to_global_pos_and_stop(temp_state.pose.pose)
        print("Finished moving back to previous global state")


        # self.update_desired_pos_global(temp_state)

        # rospy.sleep(1)
        
        # while not rospy.is_shutdown():
        #     self._pub_desired_pose.publish(temp_state.pose.pose)
        #     print(f"Yaw setpoint: {self.current_setpoint[5]}")
        #     rate.sleep()

        #     if self.current_setpoint[5] == 0:
        #         continue

        #     if abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
        #         break

        # print("Finished yawing")

        # self.move_to_pos_and_stop(1.5, 0, 0)
        # print("Finished task")
    
    # def cv_gate(self):
    #     rate = rospy.Rate(15)

    #     self.move_to_pos_and_stop(0, 0, -1)
    #     print("Finished submerging")
        
    #     self.move_to_pos_and_stop(3, 0, 0)
    #     print("Finished moving forward")
        
    #     # print(f"{self.abydos_gate_yaw}")
    #     # self.move_to_pos_and_stop(0, 0, 0, yaw=self.abydos_gate_yaw)
    #     # print(f"{self.abydos_gate_yaw}")
    #     # print("Finished yawing to gate")

    #     while not rospy.is_shutdown():
    #         if self.abydos_gate_pos_x == 0 or self.abydos_gate_pos_y == 0:
    #             print("Abydos not detected")
    #             self.update_desired_pos_local(0, 0, 0)
    #             continue

    #         self.update_desired_pos_local(self.abydos_gate_pos_x, self.abydos_gate_pos_y, 0) 
    #         print(self.current_setpoint)

    #         if self.abydos_gate_pos_x < 1:
    #             print("Break due to low x")
    #             break

    #         rate.sleep()
            
    #         # if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and abs(
    #         #         self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and abs(
    #         #     self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2]:
    #         #     break
            
            
    #         if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and abs(
    #             self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1]:
    #             print("Hit setpoint")
    #             # break
            

    #     print("Hit Abydos")

    #     self.move_to_pos_and_stop(3, 0, 0)
    #     print("Finished gate")

    #     return

    #     rospy.sleep(1)
        
    #     print("Current Abydos x setpoint: ", self.current_setpoint)

    #     while True:
    #         print(self.current_setpoint)
    #         # if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and abs(
    #         #         self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and abs(
    #         #     self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2]:
    #         #     break
                
    #         if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and abs(
    #                 self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1]:
    #             break
            
    #         rate.sleep()
    #         # tsp.update_desired_pos_local(tsp.taurus_pos_x + 0.5, tsp.taurus_pos_y, tsp.taurus_pos_z)

    #     print("Hit Abydos x")

    def cv_buoy(self, initial_depth):
        rate = rospy.Rate(15)

        self.move_to_pos_and_stop(0, 0, initial_depth)
        print("Finished submerging")
        
        rospy.sleep(1)

        self.sonar_requests.publish("buoy_abydos_serpenscaput")

        self.serpenscaput_pose_transformed.position.x = self.serpenscaput_pose_transformed.position.x + 0.25
        self.serpenscaput_pose_transformed.position.y = self.serpenscaput_pose_transformed.position.y - 0.25
        self.serpenscaput_pose_transformed.position.z = self.serpenscaput_pose_transformed.position.z - 0.25
        print(self.serpenscaput_pose_transformed)
        self._pub_desired_pose.publish(self.serpenscaput_pose_transformed)

        while not rospy.is_shutdown():
            
            # if self.serpenscaput_pose_transformed.position.x == 0 or self.serpenscaput_pose_transformed.position.y == 0 \
            #     or self.serpenscaput_pose_transformed.position.z == 0:
            #         print("Ignoring taurus 0")
            #         continue
            
            self.serpenscaput_pose_transformed.position.x = self.serpenscaput_pose_transformed.position.x + 0.25
            self.serpenscaput_pose_transformed.position.z = self.serpenscaput_pose_transformed.position.z - 0.25
            self._pub_desired_pose.publish(self.serpenscaput_pose_transformed)
            print(self.current_setpoint)
            
            rate.sleep()

            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                print("Ignoring 0 0 0 setpoint")
                continue
            
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and \
            abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and \
            abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2] and \
            abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                print("reached setpoint")
                break
            
            if self.serpenscaput_time != 0 and abs(self.serpenscaput_time - rospy.Time.now().secs) > 3:
                print("did not receive detection for 3 seconds")
                break

        print("Hit Serpens Caput")

        rospy.sleep(1)
        
        # self.move_to_pos_and_stop(3, 0, 0)
        # print("Finished moving forward")

        # NOTE: CHECK ORIENTATION OF BUOY
        temp_state = copy.deepcopy(self.state)
        # temp_state.pose.pose.position.x = temp_state.pose.pose.position.x - 2
        # temp_state.pose.pose.position.x = temp_state.pose.pose.position.y + 0.5
        # temp_state.pose.pose.position.x = temp_state.pose.pose.position.x - 1

        temp_state.pose.pose.position.x = temp_state.pose.pose.position.x - 1
        temp_state.pose.pose.position.y = temp_state.pose.pose.position.y - 0.5
        temp_state.pose.pose.position.z = temp_state.pose.pose.position.z
        
        temp_state.pose.pose.orientation.x = 0
        temp_state.pose.pose.orientation.y = 0
        temp_state.pose.pose.orientation.z = 0
        temp_state.pose.pose.orientation.w = 1
        
        self.move_to_global_pos_and_stop(temp_state.pose.pose)
        print("Finished moving back and down")

        rospy.sleep(1)

        # DEAD RECKON 2ND BUOY USING PREVIOUSLY SAVED GLOBAL POSE OF THE 1ST BUOY
        # temp_state.pose.pose.position.x = temp_state.pose.pose.position.x + 1.5
        # temp_state.pose.pose.position.y = temp_state.pose.pose.position.y + 1
        # temp_state.pose.pose.position.z = temp_state.pose.pose.position.z + 0.9

        # # self.move_to_pos_and_stop(2, 0.6, 0.3)
        # self.move_to_global_pos_and_stop(temp_state.pose.pose)
        # print("Hit Taurus")

        # DEAD RECKON 2ND BUOY USING LOCAL POSE
        # self.move_to_pos_and_stop(0, 0.6, 0.3)
        # print("Finished moving left")

        # self.move_to_pos_and_stop(2, 0, 0)
        # print("Hit Taurus")

        # DETECTS 2ND BUOY AND MOVES TOWARDS IT        
        self.sonar_requests.publish("buoy_abydos_taurus")

        self.taurus_pose_transformed.position.x = self.taurus_pose_transformed.position.x + 0.5
        self.taurus_pose_transformed.position.y = self.taurus_pose_transformed.position.y - 0.25
        self.taurus_pose_transformed.position.z = self.taurus_pose_transformed.position.z - 0.25
        print(self.taurus_pose_transformed)
        self._pub_desired_pose.publish(self.taurus_pose_transformed)
        
        while not rospy.is_shutdown():
            # if self.taurus_pose_transformed.position.x == 0 or self.taurus_pose_transformed.position.y == 0 \
            #     or self.taurus_pose_transformed.position.z == 0:
            #         print("Ignoring taurus 0")
            #         continue
            
            # if abs(self.current_setpoint[1] - self.taurus_pos_y) > 0.1:
            #     print(f"diff    : {abs(self.current_setpoint[1] - self.taurus_pos_y)}")
            # print(f"taurus y: {self.taurus_pose_transformed.position.y}")
            # print(f"diff    : {abs(self.current_setpoint[1] - self.taurus_pos_y)}")
            # print()

            self.taurus_pose_transformed.position.x = self.taurus_pose_transformed.position.x + 0.5
            self.taurus_pose_transformed.position.z = self.taurus_pose_transformed.position.z - 0.25
            self._pub_desired_pose.publish(self.taurus_pose_transformed)
            print(self.current_setpoint)
            
            rate.sleep()

            if self.current_setpoint[0] == 0 or self.current_setpoint[1] == 0 or self.current_setpoint[2] == 0:
                print("Ignoring 0 0 0 setpoint")
                continue
            
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and \
            abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and \
            abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2] and \
            abs(self.current_setpoint[5]) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                print("reached setpoint")
                break
            
            if self.taurus_time != 0 and abs(self.taurus_time - rospy.Time.now().secs) > 3:
                print("did not receive detection for 3 seconds")
                break
                
            # if self.abydos_gate_pos_x > 20:
            #     print("X value too high breaking...")
            #     break
        
        print("Hit Taurus")

        print("Passed semi-finals :)")

def buoy_dead_reckon(self):
    print("Starting CV buoy dead reckon")

    self.move_to_pos_and_stop(0, 0, -1)
    print("Finished submerging")
    
    self.move_to_pos_and_stop(1.5, 0, 0)
    print("Finished moving forward")

    self.move_to_pos_and_stop(-1, 0, 0)
    print("Moving backward")
    
    self.move_to_pos_and_stop(1, 0, 0)
    print("Finished moving forward")
    

def main():
    # Uncomment for competition
    # rospy.sleep(10)

    tsp = TestStatePublisher()

    # DEAD RECKON GATE WITH STYLE, THEN YAW BACK TO ORIGINAL YAW, THEN SUBMERGE AND MOVE FORWARD AGAIN
    # FOR COURSE A, MOVE FORWARD 12
    # FOR COURSE B, MOVE FORWARD 9.5
    # ALSO REMEMBER TO SET THE DESIRED NUMBER OF SECONDS FOR 2 ROTATIONS
    # RIGHT NOW IT'S SET TO 13 WHICH IS SLIGHTLY OVER 2 ROTATIONS, JUST TO MAKE SURE WE HIT 2 ROTATIONS
    # tsp.dead_reckon_gate_with_style(12, -2)
    # tsp.dead_reckon_gate_with_style_with_yaw_correction(9, -2)
    # tsp.dead_reckon_gate_with_style_with_yaw_correction(1.5, -0.6)

    # CV BUOY
    tsp.cv_buoy(-0.5)
    # tsp.cv_buoy(-2.5) # if just doing buoy
    
    # tsp.buoy_dead_reckon()

    # tsp.move_to_pos_and_stop(0, 0, -1)
    # print("Finished submerging")

    # tsp.move_to_pos_and_stop(0, 0, 0, yaw=-3.14)

    # CV GATE
    # tsp.cv_gate()

    
    # rate = rospy.Rate(15)
    
    # TestStatePublisher().publish_desired_pose_global()
    # tsp.publish_desired_pose_local()
    # TestStatePublisher().publish_desired_twist()
    # TestStatePublisher().publish_desired_power()
    # TestStatePublisher().move_to_pos_and_stop()


if __name__ == '__main__':
    main()
