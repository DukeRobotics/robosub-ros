#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from thruster_manager import ThrusterManager


def main():
    try:
        controls()
    except rospy.ROSInterruptException:
        pass

def controls():

    # ROS setup
    pub_to_sim = rospy.Publisher('/sim/move', Float32MultiArray, queue_size=10)
    rospy.init_node('thruster_controls') # anonymous=True?
    rate = rospy.Rate(2)  # 2 Hz

    # Thruster Manager
    tm = ThrusterManager('cthulhu.config')
    print(tm.wrenchmat)
    print(tm.wrenchmat_pinv)

    # ROS Pub/Sub loop
    while not rospy.is_shutdown():
        t_allocs = tm.calc_thruster_allocs(np.random.normal(0, 1, 6))
        f32_t_allocs = Float32MultiArray()
        f32_t_allocs.data = t_allocs
        rospy.loginfo(f32_t_allocs)
        pub_to_sim.publish(f32_t_allocs)
        rate.sleep()


if __name__ == '__main__':
    main()
