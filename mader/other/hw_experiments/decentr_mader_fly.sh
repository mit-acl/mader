#!/bin/bash

# this script creates multiple windows for fly and mader
# referred to fly script used for voxl setup, which is in /extras

# if deleting old .bag and .active in voxl and .txt on nuc
ifDELETE=$1

# session name
SESSION=decentr_rmader
WINDOW=base_station

# creates the session with a name and renames the window name
cmd="new-session -d -s $SESSION -x- -y-; rename-window $WINDOW"
tmux -2 $cmd

# window number
w=0

###split tmux into 2x6

for i in {1..6} #instead of {1..5}, we need another pane for roscore (which will be run inside of one of the NUCs)
do
	tmux split-window -h
	tmux select-layout -t $SESSION:$w.$i even-horizontal
	tmux select-pane -t $SESSION:$w.$i
done

for i in 0 2 4 6 8 10
do 
	tmux select-pane -t $SESSION:$w.$i
	tmux split-window -v
done

for i in 1 3 5 7 9 11
do
	tmux resize-pane -t $SESSION:$w.$i -y 20
done

# for roscore
tmux send-keys -t $SESSION:$w.12 "ssh root@nx03.local" C-m
sleep 3
tmux send-keys -t $SESSION:$w.12 "ssh nuc3@192.168.100.3" C-m
sleep 3
tmux send-keys -t $SESSION:$w.12 "roscore" C-m

# wait for .bashrc to load
sleep 5

# send commands to each pane
# ssh each voxl
for i in {0..1}
do 
	tmux send-keys -t $SESSION:$w.$((0+i)) "ssh root@nx01.local" C-m
	tmux send-keys -t $SESSION:$w.$((2+i)) "ssh root@nx02.local" C-m
	tmux send-keys -t $SESSION:$w.$((4+i)) "ssh root@nx03.local" C-m
	tmux send-keys -t $SESSION:$w.$((6+i)) "ssh root@nx04.local" C-m
	tmux send-keys -t $SESSION:$w.$((8+i)) "ssh root@nx05.local" C-m
	tmux send-keys -t $SESSION:$w.$((10+i)) "ssh root@nx06.local" C-m
done

sleep 5

for i in 0 2 4 6 8 10 
do
	tmux send-keys -t $SESSION:$w.$i "./ad_hoc_connection.sh" C-m
done

sleep 3

# ssh each nuc from voxl 
tmux send-keys -t $SESSION:$w.1 "ssh nuc1@192.168.100.1" C-m
tmux send-keys -t $SESSION:$w.3 "ssh nuc2@192.168.100.2" C-m
tmux send-keys -t $SESSION:$w.5 "ssh nuc3@192.168.100.3" C-m
tmux send-keys -t $SESSION:$w.7 "ssh nuc4@192.168.100.4" C-m
tmux send-keys -t $SESSION:$w.9 "ssh nuc5@192.168.100.5" C-m
tmux send-keys -t $SESSION:$w.11 "ssh nuc6@192.168.100.6" C-m


sleep 1

# send commands

# rm bags and fly
for i in 0 2 4 6 8 10 
do
	if [[ $ifDELETE == 'true' ]]; then
		tmux send-keys -t $SESSION:$w.$i "cd /data/bags && rm *.bag && rm *.active" C-m
		sleep 1
	fi
	# tmux send-keys -t $SESSION:$w.$i "tmux kill-server" C-m
	tmux send-keys -t $SESSION:$w.$i "fly" C-m
done

# run mader hw_onboard and save termial data into txt files
if [[ $ifDELETE == 'true' ]]; then 
	tmux send-keys -t $SESSION:$w.1 "cd /home/nuc1/Research/bags && rm *.txt" C-m
	tmux send-keys -t $SESSION:$w.3 "cd /home/nuc2/Research/bags && rm *.txt" C-m
	tmux send-keys -t $SESSION:$w.5 "cd /home/nuc3/Research/bags && rm *.txt" C-m
	tmux send-keys -t $SESSION:$w.7 "cd /home/nuc4/Research/bags && rm *.txt" C-m
	tmux send-keys -t $SESSION:$w.9 "cd /home/nuc5/Research/bags && rm *.txt" C-m
	tmux send-keys -t $SESSION:$w.11 "cd /home/nuc6/Research/bags && rm *.txt" C-m
fi

# run mader hw_onboard and save termial data into txt files
tmux send-keys -t $SESSION:$w.1 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.3 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.5 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.7 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.9 "sudo ntpdate time.nist.gov" C-m
tmux send-keys -t $SESSION:$w.11 "sudo ntpdate time.nist.gov" C-m

sleep 5

tmux send-keys -t $SESSION:$w.1 "(roscd mader && git rev-parse HEAD && git diff --color && cd /home/nuc1/Research/bags/ && roslaunch mader hw_onboard.launch quad:=NX01) 2>&1 | tee ~/Research/bags/nx01_mader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m #by using /home/nuc1/ instead of ~/, we can stop record data on sikorsky when we are not using the vehicle.
tmux send-keys -t $SESSION:$w.3 "(roscd mader && git rev-parse HEAD && git diff --color && cd /home/nuc2/Research/bags/ && roslaunch mader hw_onboard.launch quad:=NX02) 2>&1 | tee ~/Research/bags/nx02_mader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
tmux send-keys -t $SESSION:$w.5 "(roscd mader && git rev-parse HEAD && git diff --color && cd /home/nuc3/Research/bags/ && roslaunch mader hw_onboard.launch quad:=NX03) 2>&1 | tee ~/Research/bags/nx03_mader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
tmux send-keys -t $SESSION:$w.7 "(roscd mader && git rev-parse HEAD && git diff --color && cd /home/nuc4/Research/bags/ && roslaunch mader hw_onboard.launch quad:=NX04) 2>&1 | tee ~/Research/bags/nx04_mader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
tmux send-keys -t $SESSION:$w.9 "(roscd mader && git rev-parse HEAD && git diff --color && cd /home/nuc5/Research/bags/ && roslaunch mader hw_onboard.launch quad:=NX05) 2>&1 | tee ~/Research/bags/nx05_mader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m
tmux send-keys -t $SESSION:$w.11 "(roscd mader && git rev-parse HEAD && git diff --color && cd /home/nuc6/Research/bags/ && roslaunch mader hw_onboard.launch quad:=NX06) 2>&1 | tee ~/Research/bags/nx06_mader_$(date '+%Y_%m_%d_%H_%M_%S').txt" C-m

tmux -2 attach-session -t $SESSION