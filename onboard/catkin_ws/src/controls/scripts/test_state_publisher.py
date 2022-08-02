#!/usr/bin/env python3

from copy import deepcopy
import secrets
import rospy
from custom_msgs.msg import CVObject
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import controls_utils
from tf import TransformListener
from time import sleep
import math

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
        self.state_listener = rospy.Subscriber("/controls/yaw_pos/setpoint", Float64, self._on_receive_data_yaw)

        self.cv_listener_gman = rospy.Subscriber("/cv/right/buoy_gman", CVObject, self._on_receive_buoy_gman)
        self.cv_listener_bootlegger = rospy.Subscriber("/cv/right/buoy_bootlegger", CVObject, self._on_receive_buoy_bootlegger)

        self.current_setpoint = [100.0, 100.0, 100.0] # x,y,z
        self.MOVE_OFFSET_CONSTANT = 1
        self.current_yaw = 100.0
        self.MOVE_OFFSET_CONSTANT_ANGULAR = 0.2

        self.gman_xmin = 0.0
        self.gman_ymin = 0.0
        self.gman_xmax = 0.0
        self.gman_ymax = 0.0

        self.bootlegger_xmin = 0.0
        self.bootlegger_ymin = 0.0
        self.bootlegger_xmax = 0.0
        self.bootlegger_ymax = 0.0

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
        self.desired_pose_local.position.x = 0
        self.desired_pose_local.position.y = 0
        self.desired_pose_local.position.z = 0
        self.desired_pose_local.orientation.x = 0
        self.desired_pose_local.orientation.y = 0
        self.desired_pose_local.orientation.z = 0
        self.desired_pose_local.orientation.w = 1
        self.recalculate_local_pose()
        
        self.global_start_pose = Pose()

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

    # Publish self.desired_twist for a fixed duration
    def publish_desired_twist_duration(self, duration):
        rate = rospy.Rate(15)
        tot = 0
        while not rospy.is_shutdown() and tot < duration:
            self._pub_desired_twist.publish(self.desired_twist)
            # self._pub_current_state.publish(self.current_state)
            rate.sleep()
            tot += 15

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
        #self.desired_pose_local.orientation.x = 0
        #self.desired_pose_local.orientation.y = 0
        #self.desired_pose_local.orientation.z = 0
        #self.desired_pose_local.orientation.w = 1

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

    def move_to_yaw_and_stop(self, x, y, z, w):
        #self.desired_pose_local.position.x = 0
        #self.desired_pose_local.position.y = 0
        self.desired_pose_local.position.z = -0.1
        self.desired_pose_local.orientation.x = x
        self.desired_pose_local.orientation.y = y
        self.desired_pose_local.orientation.z = z
        self.desired_pose_local.orientation.w = w

        self.recalculate_local_pose()

        rate = rospy.Rate(15)

        delay = 0
        while not rospy.is_shutdown():
            delay += 1
            self._pub_desired_pose.publish(self.desired_pose_transformed)

            if delay > 30:
                if abs(self.current_yaw) <= self.MOVE_OFFSET_CONSTANT_ANGULAR and abs(self.current_yaw) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                    break
            rate.sleep()
    
    # Point the robot forward in the global frame
    def global_face_foward(self):
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
                if abs(self.current_yaw) <= self.MOVE_OFFSET_CONSTANT_ANGULAR and abs(self.current_yaw) <= self.MOVE_OFFSET_CONSTANT_ANGULAR:
                    break
            rate.sleep()
        


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


        #initial delay
        delay = 0
        sec_to_wait = 90
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            delay += 1
            if delay % 15 == 0:
                print(delay / 15)
            if delay > 15*sec_to_wait:
                break
            rate.sleep()
        print("starting!")
        
        self.move_to_pos_and_stop(5,0,-1.5)
        print("Move forward done")
        self.publish_desired_twist()

        #self.surface_in_octagon()
        
        #z_surface = -z_submerge #should cause robot to reach surface, provided robot is positively buoyant
        #self.move_to_pos_and_stop(0,0,z_surface)
        #print("Rise to surface done")

    #Then do v2 and v3; ideally we do v3 successfully and earn 2350 points total

    def last_chance(self):
        # Dear god please work

        #initial delay
        delay = 0
        sec_to_wait = 90
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            delay += 1
            if delay % 15 == 0:
                print(delay / 15)
            if delay > 15*sec_to_wait:
                break
            rate.sleep()
        print("starting!")
        
        # Save start pose
        self.desired_pose_local.position.x = 0
        self.desired_pose_local.position.y = 0
        self.desired_pose_local.position.z = 0
        self.desired_pose_local.orientation.x = 0
        self.desired_pose_local.orientation.y = 0
        self.desired_pose_local.orientation.z = 0
        self.desired_pose_local.orientation.w = 1
        self.recalculate_local_pose()
        self.global_start_pose = deepcopy(self.desired_pose_transformed)

        # Move through the gate
        self.move_to_pos_and_stop(5,0,-1.5)
        print("Move forward done")
        # Spin for the listed duration
        self.publish_desired_twist_duration(45_000)

        # Face towards the octagon
        self.global_face_forward()

        # Move to the octagon
        self.move_to_pos_and_stop(17, 0, 0)

        # Surface in the octagon
        self.move_to_pos_and_stop(0, 0, 100)

    
    def test_comm_cv(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            print(self.bootlegger_xmin)
            rate.sleep()

    def calculate_depth_to_gman_buoy(self,d):
        f = 1347.67 #y focal length in pixels for camera
        D = 1.2192 #height of buoy in meters
        return f * D / d #calculated x distance to buoy using similar triangles

    def move_to_gman_buoy(self):
        horizontal_fov = 50
        horizontal_pixel_count = 1210
        degrees_per_pixel_horizontal = horizontal_fov / horizontal_pixel_count

        vertical_fov = 0
        vertical_pixel_count = 760
        degrees_per_pixel_vertical = vertical_fov / vertical_pixel_count

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            d = (self.gman_ymax - self.gman_ymin) * 760 #height of buoy in pixels
            x_setpoint = self.calculate_depth_to_gman_buoy(d)
            y_setpoint_in_degrees = -((self.gman_xmax + self.gman_xmin)/2 - 0.5) * 1210 * degrees_per_pixel_horizontal
            y_setpoint = x_setpoint * math.tan(y_setpoint_in_degrees)
            z_setpoint_in_degrees = -((self.gman_ymax + self.gman_ymin)/2 - 0.5) * 760 * degrees_per_pixel_vertical
            z_setpoint = x_setpoint * math.tan(z_setpoint_in_degrees)
            self.desired_pose_local.position.x = x_setpoint
            self.desired_pose_local.position

            rate.sleep()




    def _on_receive_data_x(self, data):
        self.current_setpoint[0] = data.data

    def _on_receive_data_y(self, data):
        self.current_setpoint[1] = data.data

    def _on_receive_data_z(self, data):
        self.current_setpoint[2] = data.data

    def _on_receive_data_yaw(self, data):
        self.current_yaw = data.data


    def _on_receive_buoy_gman(self, data):
        if data.score > 0.7:
            self.gman_xmin = data.xmin
            self.gman_xmax = data.xmax
            self.gman_ymin = data.ymin
            self.gman_ymax = data.ymax

    def _on_receive_buoy_bootlegger(self, data):
        if data.score > 0.7:
            self.bootlegger_xmin = data.xmin
            self.bootlegger_xmax = data.xmax
            self.bootlegger_ymin = data.ymin
            self.bootlegger_ymax = data.ymax

     

def main():
    #TestStatePublisher().gate_move()
    # TestStatePublisher().publish_desired_pose_global()
    TestStatePublisher().semifinal_sunday_v1()
    # TestStatePublisher().publish_desired_pose_local()
    # TestStatePublisher().move_to_pos_and_stop(3,0,-1.2)
    # TestStatePublisher().publish_desired_twist()
    # TestStatePublisher().publish_desired_power()
    # TestStatePublisher().test_yaw()
    #TestStatePublisher().move_to_pos_and_stop(8,0,-1)
    #TestStatePublisher().move_to_yaw_and_stop(0,0,1,0) #submerges as well
    #print("Done with yaw")
    #TestStatePublisher().move_to_pos_and_stop(2,0,-0.5)
    print("Done with x")

    #TestStatePublisher().semifinal_sunday_v1()
    #TestStatePublisher().test_comm_cv()

    


if __name__ == '__main__':
    main()
