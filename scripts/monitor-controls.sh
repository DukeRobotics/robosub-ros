#!/bin/bash

# A tmux script that opens all the necessary terminals to monitor robot control.
# The left pane displays setpoints and the right pane displays control efforts.
# From top to bottom, each pair of terminals displays: x, y, z, roll, pitch, yaw.

# Usage: ./monitor.sh
# Note: This script assumes that you have already run `./pool-test.sh`.
# Specifically, the onboard docker container and roscore must be running.

session="controls"

tmux new-session -d -s $session

tmux split-window -h -p 50

tmux split-window -v -p 100
tmux split-window -v -p 80
tmux split-window -v -p 60
tmux split-window -v -p 40
tmux split-window -v -p 20

tmux select-pane -t 0
tmux split-window -v -p 100
tmux split-window -v -p 80
tmux split-window -v -p 60
tmux split-window -v -p 40
tmux split-window -v -p 20

tmux select-pane -t 0
tmux rename-window -t $session:$window 'onboard_1'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /controls/x_pos/setpoint' 'Enter'

tmux select-pane -t 1
tmux rename-window -t $session:$window 'onboard_2'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /controls/y_pos/setpoint' 'Enter'

tmux select-pane -t 2
tmux rename-window -t $session:$window 'onboard_3'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /controls/z_pos/setpoint' 'Enter'

tmux select-pane -t 3
tmux rename-window -t $session:$window 'onboard_4'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /controls/roll_pos/setpoint' 'Enter'

tmux select-pane -t 4
tmux rename-window -t $session:$window 'onboard_5'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /controls/pitch_pos/setpoint' 'Enter'

tmux select-pane -t 5
tmux rename-window -t $session:$window 'onboard_6'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /controls/yaw_pos/setpoint' 'Enter'

tmux select-pane -t 6
tmux rename-window -t $session:$window 'onboard_7'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /control_effort/x' 'Enter'

tmux select-pane -t 7
tmux rename-window -t $session:$window 'onboard_8'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /control_effort/y' 'Enter'

tmux select-pane -t 8
tmux rename-window -t $session:$window 'onboard_9'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /control_effort/z' 'Enter'

tmux select-pane -t 9
tmux rename-window -t $session:$window 'onboard_10'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /control_effort/roll' 'Enter'

tmux select-pane -t 10
tmux rename-window -t $session:$window 'onboard_11'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /control_effort/pitch' 'Enter'

tmux select-pane -t 11
tmux rename-window -t $session:$window 'onboard_12'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /control_effort/yaw' 'Enter'

tmux select-pane -t 0

tmux attach-session -t $session