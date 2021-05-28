#!/usr/bin/env python3
import rospy
from cthulhu_model import Cthulhu
from custom_msgs.msg import ThrusterSpeeds
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sim_handle import SimHandle

class SimLoop:

    ODOM_TOPIC = 'sensors/dvl/odom'
    IMU_TOPIC = 'sensors/imu/imu'
    ROBOT_MOVE_TOPIC = '/offboard/thruster_speeds'

    def __init__(self):
        rospy.init_node("simulation")

        self.thruster_command = ThrusterSpeeds().speeds
        self.sub = rospy.Subscriber(self.ROBOT_MOVE_TOPIC, ThrusterSpeeds, self.on_move_received)

        self.odom_pub = rospy.Publisher(self.ODOM_TOPIC, Odometry, queue_size=3)
        self.imu_pub = rospy.Publisher(self.IMU_TOPIC, Imu, queue_size=3)

        self.sim_handle = SimHandle()
        self.robot_model = Cthulhu(self.sim_handle.get_mass())

    def on_move_received(self, msg):
        self.thruster_command = msg.speeds
    
    def publish_imu(self, orient, ang_vel):
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'imu_link'
        msg.orientation = orient
        msg.angular_velocity = ang_vel
        self.imu_pub.publish(msg)

    def publish_odom(self, orient, lin_vel):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'dvl_link'
        msg.pose.pose = Pose(Point(), orient)
        msg.twist.twist = Twist(lin_vel, Vector3())
        self.odom_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            pose = self.sim_handle.get_pose()
            twist = self.sim_handle.get_twist()
            xsize, ysize, zsize = self.sim_handle.get_size()
            grav = self.sim_handle.get_gravity()

            self.publish_imu(pose.orientation, twist.angular)
            self.publish_odom(pose.orientation, twist.linear)

            fbuoy, dragforce = self.robot_model.get_mechanical_forces(xsize, ysize, zsize, grav, pose.position, twist.linear, twist.angular)
            tforces = self.robot_model.get_thruster_forces(self.thruster_command)

            #while self.sim_handle.get_barrier == 1:
            #    rospy.sleep(0.05)
            self.sim_handle.add_drag_force(dragforce)
            self.sim_handle.add_buoyancy_force(self.robot_model.CENTER_OF_BUOY, fbuoy)
            for i in range(len(tforces)):
                self.sim_handle.add_thruster_force(self.robot_model.THRUSTER_POINTS[i], tforces[i])
            
            #self.sim_handle.set_barrier()
            rate.sleep()

if __name__ == '__main__':
    SimLoop().run()