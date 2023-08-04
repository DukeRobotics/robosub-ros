#!/bin/bash
session=“docker”

tmux new-session -d -s $session

tmux select-pane -t 0
tmux rename-window -t $session:$window 'robot'
tmux send-keys -t $session:$window 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys -t $session:$window 'docker run -td --privileged --net=host -v /home/robot/robosub-ros:/root/dev/robosub-ros -v /dev:/dev dukerobotics/robosub-ros:onboard' 'Enter'

tmux attach-session -t $session