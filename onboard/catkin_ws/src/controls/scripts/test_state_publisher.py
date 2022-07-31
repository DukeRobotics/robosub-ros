#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import controls_utils
from tf import TransformListener
from time import sleep

class TestStatePublisher:
    PUBLISHING_TOPIC_DESIRED_POSE = 'controls/desired_pose'
    PUBLISHING_TOPIC_DESIRED_TWIST = 'controls/desired_twist'
    PUBLISHING_TOPIC_CURRENT_STATE = '/state'
    PUBLISHING_TOPIC_DESIRED_POWER = 'controls/desired_power'
    

    def recalculate_local_pose(self):
        self.desired_pose_transformed = controls_utils.transform_pose(self.listener, "base_link", "odom", self.desired_pose_local)

    def __init__(self):
        rospy.init_node('test_state_publisher')
        self.listener = TransformListener()
        self.state_listener = rospy.Subscriber("/controls/x_pos/setpoint", Float64, self._on_receive_data_x)
        self.state_listener = rospy.Subscriber("/controls/y_pos/setpoint", Float64, self._on_receive_data_y)
        self.state_listener = rospy.Subscriber("/controls/z_pos/setpoint", Float64, self._on_receive_data_z)

        self.current_setpoint = [100.0, 100.0, 100.0] # x,y,z
        self.MOVE_OFFSET_CONSTANT = 0.2



        sleep(1)

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
        self.desired_pose_local.position.x = 2
        self.desired_pose_local.position.y = 0
        self.desired_pose_local.position.z = 0
        self.desired_pose_local.orientation.x = 0
        self.desired_pose_local.orientation.y = 0
        self.desired_pose_local.orientation.z = 0
        self.desired_pose_local.orientation.w = 1
        self.recalculate_local_pose()

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
        self.desired_power.linear.x = 1
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

    def gate_move(self):
        delay = 500
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            delay -= 1
            if delay == 0:
                self.desired_pose_local.position.z = -1.2
                self.desired_pose_local.position.x = 8
                self.recalculate_local_pose()
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()

    def test_yaw(self):
        delay = 0
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            delay += 1
            if delay == 1:
                self.desired_twist.angular.z = 1
            if delay == 150:
                print("Stop")
                self.desired_twist.angular.z = 0
            if delay == 300:
                print("Switching direction")
                self.desired_twist.angular.z = -1
            self._pub_desired_twist.publish(self.desired_twist)
            rate.sleep()

    def semifinal_sunday_with_timings(self):
        delay = 0
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            delay += 1
            if delay == 1:
                print("Diving")
                self.desired_pose_local.position.z = -1.2
                self.recalculate_local_pose()
            if delay == 60:
                print("Going forward")
                self.desired_pose_local.position.z = 0
                self.desired_pose_local.position.x = 15
                self.recalculate_local_pose()
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            rate.sleep()

    def circle_pole(self):
        delay = 0
        rate = rospy.Rate(15)
        #self.desired_pose_local.position.x = 0
        while not rospy.is_shutdown():
            delay += 1
            if delay == 30:
                self.desired_pose_local.position.x = 0
                self.desired_pose_local.position.y = -2
                self.recalculate_local_pose()
            if delay == 210:
                print("Back")
                self.desired_pose_local.position.y = 0
                self.desired_pose_local.position.x = -5
                self.recalculate_local_pose()
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            rate.sleep()

    def publish_desired_twist(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_twist.publish(self.desired_twist)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()

    def publish_desired_power(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_power.publish(self.desired_power)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()

    def test_receive_data(self):
        self.desired_pose_local.position.x = 2
        self.desired_pose_local.position.y = 0
        self.desired_pose_local.position.z = 0

        self.recalculate_local_pose()
    
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            print(self.current_setpoint)
            print(type(self.current_setpoint[0]))
            if self.current_setpoint[0] <= 0.2 and self.current_setpoint[1] <= 0.2 and self.current_setpoint[2] <= 0.2:
                print("Here!")
            rate.sleep()

    def move_to_pos_and_stop(self, x, y, z):
        self.desired_pose_local.position.x = x
        self.desired_pose_local.position.y = y
        self.desired_pose_local.position.z = z

        self.recalculate_local_pose()

        rate = rospy.Rate(15)
        
        delay = 0
        while not rospy.is_shutdown():
            delay += 1
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            
            if delay > 30:
                #print(self.current_setpoint)
                #if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT and abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT and abs(self.current_setpoint[2]) <= self.MOVE_OFFSET_CONSTANT:
                if abs(self.current_setpoint[0]) <= self.MOVE_OFFSET_CONSTANT and abs(self.current_setpoint[1]) <= self.MOVE_OFFSET_CONSTANT:
                    #print("Done with loop")
                    break
            rate.sleep()
        #print("Finished")

    def surface_in_octagon(self):
        self.desired_pose_local.position.x = 0
        self.desired_pose_local.position.y = 0
        self.desired_pose_local.position.z = 10

        self.recalculate_local_pose()

        rate = rospy.Rate(15)
        
        delay = 0
        while not rospy.is_shutdown():
            delay += 1
            self._pub_desired_pose.publish(self.desired_pose_transformed)
            if delay > 100:
                break
            rate.sleep()
    
    def semifinal_sunday_v1(self):
        #No style, no random starting orientation (just point directly to the octagon)
        #z_submerge = -1 #tune by finding depth for which we can travel length of pool without surfacing (until of couse we want to)
        #self.move_to_pos_and_stop(0,0,z_submerge)
        #print("Submerge done")
        
        x_forward_to_octagon = 17.5 #tune by measurement of pool by driving robot and looking at controls setpoint, then negate it
        self.move_to_pos_and_stop(x_forward_to_octagon,0,-1)
        print("Move forward done")
        
        #z_surface = -z_submerge #should cause robot to reach surface, provided robot is positively buoyant
        #self.move_to_pos_and_stop(0,0,z_surface)
        #print("Rise to surface done")

    #Then do v2 and v3; ideally we do v3 successfully and earn 2350 points total
    
    def _on_receive_data_x(self, data):
        self.current_setpoint[0] = data.data

    def _on_receive_data_y(self, data):
        self.current_setpoint[1] = data.data

    def _on_receive_data_z(self, data):
        self.current_setpoint[2] = data.data

def main():
    #TestStatePublisher().gate_move()
    # TestStatePublisher().publish_desired_pose_global()
    # TestStatePublisher().publish_desired_pose_local()
    # TestStatePublisher().publish_desired_twist()
    # TestStatePublisher().publish_desired_power()
    # TestStatePublisher().test_yaw()
    # TestStatePublisher().move_to_pos_and_stop(17.5,0,-1)
    TestStatePublisher().semifinal_sunday_v1()


if __name__ == '__main__':
    main()
