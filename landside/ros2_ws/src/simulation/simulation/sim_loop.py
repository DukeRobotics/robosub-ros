#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from simulation.cthulhu_model import Cthulhu
from custom_msgs.msg import ThrusterSpeeds, SimObjectArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from simulation.sim_handle import SimHandle
from std_msgs.msg import Float64
import simulation.sim as sim


class SimLoop(Node):

    NODE_NAME = "simulation"
    ODOM_TOPIC = 'sensors/dvl/odom'
    IMU_TOPIC = 'sensors/imu/imu'
    ROBOT_MOVE_TOPIC = 'offboard/thruster_speeds'
    OBJ_POINTS_TOPIC = 'sim/object_points'
    DEPTH_TOPIC = 'sensors/depth'
    RUN_LOOP_RATE = 10

    def __init__(self):
        super().__init__(self.NODE_NAME)

        self.sim_handle = SimHandle(self.get_logger())
        self.robot_model = Cthulhu(self.sim_handle.get_mass())
        self.create_subscription(ThrusterSpeeds, self.ROBOT_MOVE_TOPIC, self._on_move_received, 10)

        self.odom_pub = self.create_publisher(Odometry, self.ODOM_TOPIC, 3)
        self.imu_pub = self.create_publisher(Imu, self.IMU_TOPIC, 3)
        self.sim_object_pub = self.create_publisher(SimObjectArray, self.OBJ_POINTS_TOPIC, 3)
        self.depth_pub = self.create_publisher(Float64, self.DEPTH_TOPIC, 3)

        self._on_move_received(ThrusterSpeeds())
        self.num_cycles = 0
        self.timer = self.create_timer(1/self.RUN_LOOP_RATE, self.run)

    def _on_move_received(self, msg):
        self.tforces = self.robot_model.get_thruster_forces(msg.speeds)

    def publish_imu(self, pose, twist):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation = pose.orientation
        msg.angular_velocity = twist.angular
        self.imu_pub.publish(msg)

    def publish_odom(self, pose, twist):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'dvl_link'
        msg.pose.pose = pose
        msg.twist.twist = twist
        self.odom_pub.publish(msg)

    def publish_sim_objects(self):
        self.sim_object_pub.publish(self.sim_handle.get_sim_objects())

    def publish_depth(self, pose, twist):
        msg = Float64()
        # Float64 doesn't have a header for timestamps or fields for
        # parent/child frame labels, so they aren't included.
        msg.data = pose.position.z  # Assume position is in meters
        self.depth_pub.publish(msg)

    def run(self):
        pose = self.sim_handle.get_pose(mode=sim.simx_opmode_blocking)
        twist = self.sim_handle.get_twist()

        self.publish_imu(pose, twist)
        self.publish_odom(pose, twist)
        self.publish_depth(pose, twist)
        self.publish_sim_objects()
        self.sim_handle.set_thruster_force(self.tforces)
        print(f"sim_loop.run: Running sim loop{self.num_cycles * '.'}   ", end='\r')
        self.num_cycles = (self.num_cycles + 1) % 4


def main(args=None):
    try:
        rclpy.init(args=args)
        sim = SimLoop()
        rclpy.spin(sim)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        sim.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
