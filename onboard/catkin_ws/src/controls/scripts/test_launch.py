import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(
    uuid,
    ["/root/dev/robosub-ros/onboard/catkin_ws/src/execute/launch/motion.launch"],
    args='hoge:='
)
launch.start()
rospy.loginfo("started")

rospy.sleep(20)
# 3 seconds later
launch.shutdown()