# message type (Executing roslaunch/Executing rosrun/Terminating)
uint32 msg_type

uint32 EXECUTING = 0
uint32 TERMINATING = 1

# process info
custom_msgs/RunningNode running_node_info