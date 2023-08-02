#!/usr/bin/env python3

import rospy
import actionlib
from custom_msgs.msg import ControlsDesiredPoseAction, ControlsDesiredTwistAction, ControlsDesiredPowerAction, \
    ControlsDesiredPoseGoal, ControlsDesiredTwistGoal, ControlsDesiredPowerGoal
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
import controls_utils
from tf import TransformListener
from custom_msgs.msg import CVObject
import copy


class TestStatePublisher:
    ACTION_DESIRED_POSE = 'controls/desired_pose'
    ACTION_DESIRED_TWIST = 'controls/desired_twist'
    ACTION_DESIRED_POWER = 'controls/desired_power'
    PUBLISHING_TOPIC_CURRENT_STATE = '/state'

    def recalculate_local_pose(self):
        self.desired_pose_transformed = controls_utils.transform_pose(
            self.listener, "base_link", "odom", self.desired_pose_local)

    def __init__(self):
        
        rospy.init_node('test_state_publisher')
        self.listener = TransformListener()

        rospy.Publisher("controls/desired_feature", String)
        
        rospy.Subscriber("/controls/x_pos/setpoint", Float64, self._on_receive_data_x)
        rospy.Subscriber("/controls/y_pos/setpoint", Float64, self._on_receive_data_y)
        rospy.Subscriber("/controls/z_pos/setpoint", Float64, self._on_receive_data_z)
        rospy.Subscriber("/controls/yaw_pos/setpoint", Float64, self._on_receive_data_yaw)
        rospy.Subscriber("/cv/front/buoy_abydos_serpenscaput", CVObject, self._on_receive_data_cv_serpenscaput)
        rospy.Subscriber("/cv/front/buoy_abydos_taurus", CVObject, self._on_receive_data_cv_taurus)
        rospy.Subscriber("/cv/front/gate_abydos", CVObject, self._on_receive_data_cv_gate)
        rospy.Subscriber("/state", Odometry, self._on_receive_state)

        self.current_setpoint = [100.0, 100.0, 100.0]  # x,y,z
        self.MOVE_OFFSET_CONSTANT = [0.2, 0.2, 0.2]  # x,y,z
        self.current_yaw = 100.0
        self.MOVE_OFFSET_CONSTANT_ANGULAR = 0.2

        self.listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(10))

        self._desired_pose_client = actionlib.SimpleActionClient(self.ACTION_DESIRED_POSE, ControlsDesiredPoseAction)
        self._desired_twist_client = actionlib.SimpleActionClient(self.ACTION_DESIRED_TWIST, ControlsDesiredTwistAction)
        self._desired_power_client = actionlib.SimpleActionClient(self.ACTION_DESIRED_POWER, ControlsDesiredPowerAction)
        self._pub_current_state = rospy.Publisher(self.PUBLISHING_TOPIC_CURRENT_STATE, Odometry, queue_size=3)

        self._desired_pose_client.wait_for_server()
        self._desired_twist_client.wait_for_server()
        self._desired_power_client.wait_for_server()

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
        
        self.serpenscaput_pos_x = 0
        self.serpenscaput_pos_y = 0
        self.serpenscaput_pos_z = 0

        self.abydos_gate_pos_x = 0
        self.abydos_gate_pos_y = 0
        self.abydos_gate_pos_z = 0

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

    def publish_desired_pose_global(self):
        self._desired_pose_client.send_goal(ControlsDesiredPoseGoal(pose=self.desired_pose_global))
        rospy.spin()

    def publish_desired_pose_local(self):
        print(self.desired_pose_transformed)
        self._desired_pose_client.send_goal(ControlsDesiredPoseGoal(pose=self.desired_pose_transformed))
        rospy.spin()

    def publish_desired_twist(self):
        self._desired_twist_client.send_goal(ControlsDesiredTwistGoal(twist=self.desired_twist))
        rospy.spin()

    def publish_desired_power(self, seconds=0):
        self._desired_power_client.send_goal(ControlsDesiredPowerGoal(power=self.desired_power))
        # rospy.spin()

        rospy.sleep(seconds)

    def move_to_pos_and_stop(self, x, y, z):
        self._desired_pose_client.cancel_goal()
        self.desired_pose_local.position.x = x
        self.desired_pose_local.position.y = y
        self.desired_pose_local.position.z = z

        self.recalculate_local_pose()
        self._desired_pose_client.send_goal(ControlsDesiredPoseGoal(pose=self.desired_pose_transformed))

        rospy.sleep(1)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            print(self.current_setpoint)
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and abs(
                    self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and abs(
                self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2]:
                # print("Done with loop")
                break
            rate.sleep()

    def update_desired_pos_local(self, x, y, z):
        self._desired_pose_client.cancel_goal()
        self.desired_pose_local.position.x = x
        self.desired_pose_local.position.y = y
        self.desired_pose_local.position.z = z

        self.recalculate_local_pose()
        self._desired_pose_client.send_goal(ControlsDesiredPoseGoal(pose=self.desired_pose_transformed))
        
    def update_desired_pos_global(self, state):
        self.desired_pose_global.position.x = state.pose.pose.position.x
        self.desired_pose_global.position.y = state.pose.pose.position.y
        self.desired_pose_global.position.z = state.pose.pose.position.z
        self.desired_pose_global.orientation.x = state.pose.pose.orientation.x
        self.desired_pose_global.orientation.y = state.pose.pose.orientation.y
        self.desired_pose_global.orientation.z = state.pose.pose.orientation.z
        self.desired_pose_global.orientation.w = state.pose.pose.orientation.w
        self._desired_pose_client.send_goal(ControlsDesiredPoseGoal(pose=self.desired_pose_global))

        rospy.sleep(1)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            print(self.current_setpoint)
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and abs(
                    self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and abs(
                self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2]:
                # print("Done with loop")
                break
            rate.sleep()

        print("Finished")

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

        self._desired_pose_client.send_goal(ControlsDesiredPoseGoal(pose=self.desired_pose_transformed))

        while not rospy.is_shutdown():
            print(self.current_yaw)
            if abs(self.current_yaw) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                break

            rate.sleep()

        # self._desired_pose_client.cancel_goal()

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

        self._desired_pose_client.send_goal(ControlsDesiredPoseGoal(pose=self.desired_pose_transformed))

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

            if delay > 30:
                if abs(self.current_yaw) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                    break
            rate.sleep()

        self._desired_pose_client.cancel_goal()

    def _on_receive_data_x(self, data):
        self.current_setpoint[0] = data.data

    def _on_receive_data_y(self, data):
        self.current_setpoint[1] = data.data

    def _on_receive_data_z(self, data):
        self.current_setpoint[2] = data.data

    def _on_receive_data_yaw(self, data):
        self.current_yaw = data.data

    def _on_receive_data_cv_serpenscaput(self, data):
        self.serpenscaput_pos_x = data.coords.x
        self.serpenscaput_pos_y = data.coords.y
        self.serpenscaput_pos_z = data.coords.z

    def _on_receive_data_cv_taurus(self, data):
        self.taurus_pos_x = data.coords.x
        self.taurus_pos_y = data.coords.y
        self.taurus_pos_z = data.coords.z

    def _on_receive_data_cv_gate(self, data):
        self.abydos_gate_pos_x = data.coords.x
        self.abydos_gate_pos_y = data.coords.y
        self.abydos_gate_pos_z = data.coords.z

    def _on_receive_state(self, data):
        self.state = data

    def yaw_720(self, clockwise=True):
        power = -0.5 if clockwise else 0.5

        self.desired_power.linear.x = 0
        self.desired_power.linear.y = 0
        self.desired_power.linear.z = -0.85
        self.desired_power.angular.x = 0
        self.desired_power.angular.y = 0
        self.desired_power.angular.z = power
        self.publish_desired_power(17)

        self._desired_power_client.cancel_goal()

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
        
        self._desired_pose_client.cancel_goal()
        self.yaw_task()

        print("Finished yaw, move back to previous global state")
        self.update_desired_pos_global(temp_state)

        self._desired_pose_client.cancel_goal()

    def dead_reckon_gate(self, distance, depth):
        self.move_to_pos_and_stop(0, 0, depth) # submerge
        print("Finished submerging")
        self.move_to_pos_and_stop(distance, 0, -0.5) # forward
        print("Finished moving forward")
        self._desired_pose_client.cancel_goal()
    
    def cv_gate(self):

        self.move_to_pos_and_stop(0, 0, -1)
        print("Finished submerging")
        
        self.move_to_pos_and_stop(3, 0, 0)
        print("Finished moving forward")
        
        # self.move_to_pos_and_stop(3, 0, 0) 
        # print("Finished moving forward")
        
        rate = rospy.Rate(1)

        rospy.sleep(3)

        while True:
            self.update_desired_pos_local(self.abydos_gate_pos_x, self.abydos_gate_pos_y, 0) 
            print(self.current_setpoint)

            if self.abydos_gate_pos_x == 0 or self.abydos_gate_pos_y == 0:
                print("Continued loop")
                continue

            if self.abydos_gate_pos_x < 1:
                print("Break due to low x")
                break

            rate.sleep()
            
            # if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and abs(
            #         self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and abs(
            #     self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2]:
            #     break
            
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and abs(
                self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1]:
                print("Hit setpoint")
                break
            

        print("Hit Abydos")

        self.move_to_pos_and_stop(3, 0, 0)
        print("Finished gate")

        return

        rospy.sleep(1)
        
        print("Current Abydos x setpoint: ", self.current_setpoint)

        while True:
            print(self.current_setpoint)
            # if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and abs(
            #         self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1] and abs(
            #     self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT[2]:
            #     break
                
            if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT[0] and abs(
                    self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT[1]:
                break
            
            rate.sleep()
            # tsp.update_desired_pos_local(tsp.taurus_pos_x + 0.5, tsp.taurus_pos_y, tsp.taurus_pos_z)

        print("Hit Abydos x")
        
        self._desired_pose_client.cancel_goal()
        

def main():
    # Uncomment for competition
    # rospy.sleep(30)

    tsp = TestStatePublisher()

    # CV GATE
    tsp.cv_gate()
    
    # DEAD RECKON GATE
    # tsp.dead_reckon_gate(7, -2)

    return

    # TestStatePublisher().publish_desired_pose_global()
    # tsp.publish_desired_pose_local()
    # TestStatePublisher().publish_desired_twist()
    # TestStatePublisher().publish_desired_power()
    # TestStatePublisher().move_to_pos_and_stop()


if __name__ == '__main__':
    main()
