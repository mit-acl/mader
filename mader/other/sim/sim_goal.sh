
#!/bin/bash

# this script creates multiple windows to send goals to drons
# referred to fly script used for voxl setup, which is in /extras

# session name
SESSION=sim_goal
WINDOW=base_station

# creates the session with a name and renames the window name
cmd="new-session -d -s $SESSION ; rename-window $WINDOW"
tmux -2 $cmd

# window number
w=1

# split tmux into 4x2
for i in {1..3}
do
	tmux select-pane -t $SESSION:$w.$i
	tmux split-window -v
	tmux select-layout -t $SESSION:$w.$i even-vertical
done

for i in 1 3 5 7
do
	tmux select-pane -t $SESSION:$w.$i	
	tmux split-window -h
done

# wait for .bashrc to load
sleep 1

# send goals (randomly generated or position exchange)
if [ "$1" == "pos" ]; then
	tmux send-keys -t $SESSION:$w.1 "roslaunch mader position_exchange.launch mode:=1 quad:=SQ01s" C-m
	# sleep 1
	tmux send-keys -t $SESSION:$w.2 "roslaunch mader position_exchange.launch mode:=2 quad:=SQ02s" C-m
	# sleep 1
	tmux send-keys -t $SESSION:$w.3 "roslaunch mader position_exchange.launch mode:=3 quad:=SQ03s" C-m
	# sleep 1
	tmux send-keys -t $SESSION:$w.4 "roslaunch mader position_exchange.launch mode:=4 quad:=SQ04s" C-m
	# sleep 1
	tmux send-keys -t $SESSION:$w.5 "roslaunch mader position_exchange.launch mode:=5 quad:=SQ05s" C-m
	# sleep 1
	tmux send-keys -t $SESSION:$w.6 "roslaunch mader position_exchange.launch mode:=6 quad:=SQ06s" C-m
elif [ "$1" == "ran" ]; then
	tmux send-keys -t $SESSION:$w.1 "roslaunch mader random_goal.launch quad:=SQ01s" C-m
	sleep 1
	tmux send-keys -t $SESSION:$w.2 "roslaunch mader random_goal.launch quad:=SQ02s" C-m
	sleep 1
	tmux send-keys -t $SESSION:$w.3 "roslaunch mader random_goal.launch quad:=SQ03s" C-m
	sleep 1
	tmux send-keys -t $SESSION:$w.4 "roslaunch mader random_goal.launch quad:=SQ04s" C-m
	sleep 1
	tmux send-keys -t $SESSION:$w.5 "roslaunch mader random_goal.launch quad:=SQ05s" C-m
	sleep 1
	tmux send-keys -t $SESSION:$w.6 "roslaunch mader random_goal.launch quad:=SQ06s" C-m
fi

tmux send-keys -t $SESSION:$w.8 "tmux kill-server"

tmux -2 attach-session -t $SESSION