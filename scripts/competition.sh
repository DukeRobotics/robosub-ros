#!/bin/bash
session=“competition”

tmux new-session -d -s $session

# Split into 4 equal vertical panes
tmux split-window -h -p 75
tmux split-window -h -p 66
tmux split-window -h -p 50

tmux select-pane -t 0
tmux rename-window -t $session:$window 'onboard_1'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'roslaunch execute motion.launch'

tmux select-pane -t 1
tmux rename-window -t $session:$window 'onboard_2'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo state'

tmux select-pane -t 2
tmux rename-window -t $session:$window 'onboard_3'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rosservice call /enable_controls 1'

tmux select-pane -t 3
tmux rename-window -t $session:$window 'onboard_4'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rosrun controls test_state_publisher.py'

tmux select-pane -t 0

tmux attach-session -t $session