import roslaunch
import rospy
import argparse
import controls_utils as utils


def _get_pid_keys(pid_type="vel"):
    return [axis + '_' + pid_type + '_' + constant
            for constant in ['Kp', 'Ki', 'Kd']
            for axis in utils.get_axes()]


def _get_pid_launch_args(pid_values, pid_type="vel"):
    pid_keys = _get_pid_keys(pid_type)
    launch_args = []
    for i in range(len(pid_values)):
        launch_args.append(pid_keys[i] + ':=' + pid_values[i])
    return launch_args


# Extract CMD Line Arguments
parser = argparse.ArgumentParser()
parser.add_argument('--vel_pid', help="List of 18 PID constants", nargs='+')
args = parser.parse_args()
pid_launch_args = _get_pid_launch_args(args.vel_pid)

# Launch execute motion.launch, and pass launch arguments
rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(
    uuid,
    [("/root/dev/robosub-ros/onboard/catkin_ws/src/execute/launch/motion.launch", pid_launch_args)],
)
launch.start()
rospy.loginfo("started")

rospy.sleep(20)
# 3 seconds later
launch.shutdown()
