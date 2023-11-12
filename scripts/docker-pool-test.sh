#!/bin/bash
# shellcheck disable=SC2016

# A tmux script that starts onboard and landside docker containers,
# then opens all the necessary terminals to control our robot.
# Usage: ./docker-pool-test.sh

session="pool-test"

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
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'docker run -td --privileged --net=host -e ROBOT_NAME=oogway -v /home/robot/robosub-ros:/root/dev/robosub-ros -v /dev:/dev dukerobotics/robosub-ros:onboard' 'Enter'
tmux send-keys 'cd robosub-ros' 'Enter'
tmux send-keys 'docker run --privileged --net=host -td -p 2201:2201 -v ${PWD}:/root/dev/robosub-ros dukerobotics/robosub-ros:landside' 'Enter'

tmux select-pane -t 1
tmux send-keys 'sleep 5' C-m
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'roscore' 'Enter'

tmux select-pane -t 2
tmux send-keys 'sleep 5' C-m
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'roslaunch execute motion.launch'

tmux select-pane -t 3
tmux send-keys 'sleep 5' C-m
tmux send-keys 'ssh -XY robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -XY -p 2201 root@192.168.1.1' 'Enter'
tmux send-keys 'roslaunch gui cv_gui.launch'

tmux select-pane -t 4
tmux send-keys 'sleep 5' C-m
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rosservice call /enable_controls True'

tmux select-pane -t 5
tmux send-keys 'sleep 5' C-m
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys './record.sh .bag'

tmux select-pane -t 6
tmux send-keys 'sleep 5' C-m
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rostopic echo /state'

tmux select-pane -t 7
tmux send-keys 'sleep 5' C-m
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rosrun controls test_state_publisher.py'

tmux select-pane -t 1

tmux attach-session -t $session