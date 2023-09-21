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
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /controls/x_pos/setpoint' 'Enter'

tmux select-pane -t 1
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /controls/y_pos/setpoint' 'Enter'

tmux select-pane -t 2
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /controls/z_pos/setpoint' 'Enter'

tmux select-pane -t 3
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /controls/roll_pos/setpoint' 'Enter'

tmux select-pane -t 4
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /controls/pitch_pos/setpoint' 'Enter'

tmux select-pane -t 5
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /controls/yaw_pos/setpoint' 'Enter'

tmux select-pane -t 6
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /control_effort/x' 'Enter'

tmux select-pane -t 7
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /control_effort/y' 'Enter'

tmux select-pane -t 8
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /control_effort/z' 'Enter'

tmux select-pane -t 9
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /control_effort/roll' 'Enter'

tmux select-pane -t 10
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /control_effort/pitch' 'Enter'

tmux select-pane -t 11
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /control_effort/yaw' 'Enter'

tmux select-pane -t 0

tmux attach-session -t $session