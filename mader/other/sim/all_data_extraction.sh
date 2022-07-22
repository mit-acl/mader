#!/bin/bash

# this script creates multiple windows to extract data

# if deleting old .bag and .active in voxl and .txt on nuc

# session name
SESSION=data_extraction
WINDOW=window

# creates the session with a name and renames the window name
cmd="new-session -d -s $SESSION -x- -y-; rename-window $WINDOW"
tmux -2 $cmd

# window number
w=0

#split tmux into 2x6
for i in 1 2 3
do
	tmux split-window -h
	tmux select-layout -t $SESSION:$w.$i even-horizontal
	tmux select-pane -t $SESSION:$w.$i
done

for i in 0 2 4
do 
	tmux select-pane -t $SESSION:$w.$i
	tmux split-window -v
done

# for i in 0 2 4 6 8 10
# do
# 	tmux resize-pane -t $SESSION:$w.$i -y 300
# done

# wait for .bashrc to load
sleep 1

# send commands to each pane

# run mader hw_onboard and save termial data into txt files

for i in {1..5}
do
	tmux send-keys -t $SESSION:$w.$i "roscd mader && cd other/sim" C-m
done

sleep 1

# run mader hw_onboard and save termial data into txt files

tmux send-keys -t $SESSION:$w.0 "python ave_distance_csv2txt.py" C-m 
tmux send-keys -t $SESSION:$w.1 "python collision_check.py" C-m 
tmux send-keys -t $SESSION:$w.2 "python comm_delay_histogram_ercentlie.py" C-m 
tmux send-keys -t $SESSION:$w.3 "python completion_time.py" C-m 
tmux send-keys -t $SESSION:$w.4 "python missed_msgs_count.py" C-m

tmux -2 attach-session -t $SESSION