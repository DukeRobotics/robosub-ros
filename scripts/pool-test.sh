#!/bin/bash

# A tmux script that opens all the necessary terminals to control our robot.
# Usage: ./pool-test.sh

session=“pool-test”

tmux new-session -d -s $session

tmux split-window -h -p 50
tmux split-window -v -p 0
tmux select-pane -t 0
tmux split-window -v -p 100
tmux split-window -v -p 83
tmux split-window -v -p 66
tmux split-window -v -p 50
tmux split-window -v -p 33

tmux select-pane -t 0
tmux rename-window -t $session:$window 'robot'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'docker run -td --privileged --net=host -v /home/robot/robosub-ros:/root/dev/robosub-ros -v /dev:/dev dukerobotics/robosub-ros:onboard' 'Enter'
tmux send-keys -t $session:$window 'cd robosub-ros' 'Enter'
tmux send-keys -t $session:$window 'docker run --privileged --net=host -td -p 2201:2201 -v ${PWD}:/root/dev/robosub-ros dukerobotics/robosub-ros:landside' 'Enter'

tmux select-pane -t 1
tmux rename-window -t $session:$window 'onboard_1'
tmux send-keys -t $session:$window 'sleep 5' C-m
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'roscore' 'Enter'

tmux select-pane -t 2
tmux rename-window -t $session:$window 'onboard_2'
tmux send-keys -t $session:$window 'sleep 5' C-m
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'roslaunch execute motion.launch'

tmux select-pane -t 3
tmux rename-window -t $session:$window 'landside_1'
tmux send-keys -t $session:$window 'sleep 5' C-m
tmux send-keys -t $session:$window 'ssh -XY robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -XY -p 2201 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'roslaunch gui cv_gui.launch'

tmux select-pane -t 4
tmux rename-window -t $session:$window 'onboard_3'
tmux send-keys -t $session:$window 'sleep 5' C-m
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rosservice call /enable_controls True'

tmux select-pane -t 5
tmux rename-window -t $session:$window 'onboard_4'
tmux send-keys -t $session:$window 'sleep 5' C-m
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window './record.sh .bag'

tmux select-pane -t 6
tmux rename-window -t $session:$window 'onboard_5'
tmux send-keys -t $session:$window 'sleep 5' C-m
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rostopic echo /state'

tmux select-pane -t 7
tmux rename-window -t $session:$window 'onboard_6'
tmux send-keys -t $session:$window 'sleep 5' C-m
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'rosrun controls test_state_publisher.py'

tmux select-pane -t 1

tmux attach-session -t $session