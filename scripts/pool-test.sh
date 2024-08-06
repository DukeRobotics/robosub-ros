#!/bin/bash

# A tmux script that opens all the necessary terminals to control our robot.
# Usage: ./pool-test.sh

session="pool-test"

tmux new-session -d -s $session

tmux split-window -h
tmux split-window -h
tmux select-layout even-horizontal

tmux select-pane -t 0
tmux split-window -v -l 50

tmux select-pane -t 2
tmux split-window -v -l 50

tmux select-pane -t 4
tmux split-window -v -l 50

tmux select-pane -t 0
tmux send-keys 'tmux kill-server'

tmux select-pane -t 2
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'onboard' 'Enter'
tmux send-keys 'sleep 1' C-m # need shorter delay here so that roscore is started first
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'roscore' 'Enter'

tmux select-pane -t 4
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sleep 2' C-m
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'fg-ws'

tmux select-pane -t 1
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sleep 2' C-m
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'roslaunch execute motion.launch enable_recording:=true'

tmux select-pane -t 3
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sleep 2' C-m
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'roslaunch task_planning task_runner.launch'

tmux select-pane -t 5
tmux send-keys 'ssh robot@192.168.1.1' 'Enter'
tmux send-keys 'sleep 2' C-m
tmux send-keys 'sshpass -p robotics ssh -p 2200 root@192.168.1.1' 'Enter'
tmux send-keys 'rosservice call /controls/enable True'

tmux select-pane -t 1

tmux attach-session -t $session