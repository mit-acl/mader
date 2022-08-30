#!/bin/bash

# we need this script to git pull
# before we run ./adhoc_network.sh in NUC, it can connect to internet so we need a separate file (from decentr_mader_fly.sh) that will only do git pull

# session name
SESSION=decentr_gitpull
WINDOW=base_station

# creates the session with a name and renames the window name
cmd="new-session -d -s $SESSION ; rename-window $WINDOW"
tmux -2 $cmd

# window number
w=0

# split tmux into 10x2
for i in {0..4}
do
	tmux select-pane -t $SESSION:$w.$i
	tmux split-window -v
	tmux select-layout -t $SESSION:$w.$i even-vertical
done

for i in 0 2 4 6 8 10
do
	tmux select-pane -t $SESSION:$w.$i	
	tmux split-window -h
done

# wait for .bashrc to load
sleep 1

tmux send-keys -t $SESSION:$w.0 "ssh root@nx01.local" C-m
tmux send-keys -t $SESSION:$w.1 "ssh root@nx02.local" C-m
tmux send-keys -t $SESSION:$w.2 "ssh root@nx03.local" C-m
tmux send-keys -t $SESSION:$w.3 "ssh root@nx04.local" C-m
tmux send-keys -t $SESSION:$w.4 "ssh root@nx05.local" C-m
tmux send-keys -t $SESSION:$w.5 "ssh root@nx06.local" C-m
tmux send-keys -t $SESSION:$w.6 "ssh root@nx07.local" C-m
tmux send-keys -t $SESSION:$w.7 "ssh root@nx08.local" C-m
tmux send-keys -t $SESSION:$w.8 "ssh root@nx09.local" C-m
tmux send-keys -t $SESSION:$w.9 "ssh root@nx10.local" C-m

sleep 5

tmux send-keys -t $SESSION:$w.0 "./nuc_voxl_connection" C-m
tmux send-keys -t $SESSION:$w.1 "./nuc_voxl_connection" C-m
tmux send-keys -t $SESSION:$w.2 "./nuc_voxl_connection" C-m
tmux send-keys -t $SESSION:$w.3 "./nuc_voxl_connection" C-m
tmux send-keys -t $SESSION:$w.4 "./nuc_voxl_connection" C-m
tmux send-keys -t $SESSION:$w.5 "./nuc_voxl_connection" C-m
tmux send-keys -t $SESSION:$w.6 "./nuc_voxl_connection" C-m
tmux send-keys -t $SESSION:$w.7 "./nuc_voxl_connection" C-m
tmux send-keys -t $SESSION:$w.8 "./nuc_voxl_connection" C-m
tmux send-keys -t $SESSION:$w.9 "./nuc_voxl_connection" C-m

sleep 3

# ssh each nuc
tmux send-keys -t $SESSION:$w.0 "ssh nuc1@192.168.15.2" C-m
tmux send-keys -t $SESSION:$w.1 "ssh nuc2@192.168.16.2" C-m
tmux send-keys -t $SESSION:$w.2 "ssh nuc3@192.168.17.2" C-m
tmux send-keys -t $SESSION:$w.3 "ssh nuc4@192.168.18.2" C-m
tmux send-keys -t $SESSION:$w.4 "ssh nuc5@192.168.19.2" C-m
tmux send-keys -t $SESSION:$w.5 "ssh nuc6@192.168.20.2" C-m
tmux send-keys -t $SESSION:$w.6 "ssh nuc@192.168.21.2" C-m
tmux send-keys -t $SESSION:$w.7 "ssh nuc08@192.168.22.2" C-m
tmux send-keys -t $SESSION:$w.8 "ssh nuc9@192.168.23.2" C-m
tmux send-keys -t $SESSION:$w.9 "ssh nuc10@192.168.24.2" C-m

sleep 5

# git pull origin
tmux send-keys -t $SESSION:$w.0 "roscd mader && git pull origin && catkin build" C-m
tmux send-keys -t $SESSION:$w.1 "roscd mader && git pull origin && catkin build" C-m
tmux send-keys -t $SESSION:$w.2 "roscd mader && git pull origin && catkin build" C-m
tmux send-keys -t $SESSION:$w.3 "roscd mader && git pull origin && catkin build" C-m
tmux send-keys -t $SESSION:$w.4 "roscd mader && git pull origin && catkin build" C-m
tmux send-keys -t $SESSION:$w.5 "roscd mader && git pull origin && catkin build" C-m
tmux send-keys -t $SESSION:$w.6 "roscd mader && git pull origin && catkin build" C-m
tmux send-keys -t $SESSION:$w.7 "roscd mader && git pull origin && catkin build" C-m
tmux send-keys -t $SESSION:$w.8 "roscd mader && git pull origin && catkin build" C-m
tmux send-keys -t $SESSION:$w.9 "roscd mader && git pull origin && catkin build" C-m

tmux -2 attach-session -t $SESSION