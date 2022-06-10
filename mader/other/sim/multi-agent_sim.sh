#!/bin/bash

# this script opens tmux to do multi-agent simulations 

# number of agents 
n_agents=$1
ifDELETE=$2
# 

# session name
SESSION=multi_agent
WINDOW=multi_agent_w

# window number
w=1

# kill server
# tmux send-keys -t $SESSION:$w.$1 "tmux kill-server" C-m

# creates the session with a name and renames the window name
cmd="new-session -d -s $SESSION -x- -y-; rename-window $WINDOW"
tmux -2 $cmd

#split tmux into 2x6
for i in 1
do
	tmux split-window -h
	tmux select-layout -t $SESSION:$w.$i even-horizontal
	tmux select-pane -t $SESSION:$w.$i
done

# for i in 1 3
# do 
# 	tmux select-pane -t $SESSION:$w.$i
# 	tmux split-window -v
# done

# wait for .bashrc to load
sleep 1

# send commands to each pane

# run mader hw_onboard and save termial data into txt files

# if [[ $ifDELETE == 'true' ]]; then 
# 	tmux send-keys -t $SESSION:$w.1 "cd ~/Research/data/txt_files && rm *.txt" C-m
# fi

sleep 1


# base state in dynamic forest (If we wanna include obstacles we need to launch this. Don't forget to change the total number of agents in dynamic_forest.py)
tmux send-keys -t $SESSION:$w.1 "roslaunch mader base_station.launch type_of_environment:=dynamic_forest" C-m
# controller
tmux send-keys -t $SESSION:$w.2 "roslaunch mader many_drones.launch action:=controller" C-m
# mader
tmux send-keys -t $SESSION:$w.3 "roslaunch mader many_drones.launch action:=mader" C-m
# collision check
tmux send-keys -t $SESSION:$w.4 "roslaunch mader collision_detector.launch num_of_agents:=$n_agents" C-m
# send goals
sleep 2
tmux send-keys -t $SESSION:$w.5 "roslaunch mader many_drones.launch action:=send_goal" C-m

tmux -2 attach-session -t $SESSION