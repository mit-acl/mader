#!/usr/bin/env python
# coding=utf-8

# /* ----------------------------------------------------------------------------
#  * Copyright 2022, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import math
import os
import sys
import time
import rospy
from snapstack_msgs.msg import State
import subprocess
import numpy as np
import time
from numpy import linalg as LA
import struct

def getGoals(num_of_agents):

    formation="circle"
    goals = np.empty([num_of_agents, 3])
    radius=15;

    if(formation=="sphere"):
        # num_mer=int(math.sqrt(num_of_agents)); #Num of meridians
        # num_of_agents_per_mer=int(math.sqrt(num_of_agents));    #Num of agents per meridian
        if(num_of_agents%3==0):
            num_mer=max(int(num_of_agents/4.0),3); #Num of meridians
        else: #either divisible by 4 or will force it by changing num of agents
            num_mer=max(int(num_of_agents/4.0),4); #Num of meridians
        num_of_agents_per_mer=int(num_of_agents/num_mer);    #Num of agents per meridian

    if(formation=="circle" or formation=="square"):
        num_mer=num_of_agents
        num_of_agents_per_mer=1

    # print("num_mer= ", num_mer)
    # print("num_of_agents_per_mer= ", num_of_agents_per_mer)

    id_number=1;
    shift_z=radius;
    shift_z=1.0

    #TODO: Implement the square as well for other number_of_agents
    square_starts=[[4.0, 0.0, 1.0], 
                    [4.0, 4.0, 1.0], 
                    [0.0, 4.0, 1.0], 
                    [-4.0, 4.0, 1.0],
                    [-4.0, 0.0, 1.0],
                    [-4.0, -4.0, 1.0],
                    [0.0, -4.0, 1.0],
                    [4.0, -4.0, 1.0] ]

    square_goals=  [[-4.0, 0.0, 1.0],
                    [-4.0, -4.0, 1.0],
                    [0.0, -4.0, 1.0],
                    [4.0, -4.0, 1.0],
                    [4.0, 0.0, 1.0],
                    [4.0, 4.0, 1.0],
                    [0.0, 4.0, 1.0],
                    [-4.0, 4.0, 1.0]];

    square_yaws_deg=  [-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0];

    for i in range(1, num_mer+1):
        theta=0.0+i*(2*math.pi/num_mer);
        for j in range(1, num_of_agents_per_mer+1):

            phi=(-math.pi +j*(math.pi/(num_of_agents_per_mer+1)))
            x=radius*math.cos(theta)*math.sin(phi)
            y=radius*math.sin(theta)*math.sin(phi)
            z=shift_z + radius*math.cos(phi)

            pitch=0.0;
            roll=0.0;
            yaw= theta#+math.pi  

            goal_x=radius*math.cos(theta+2*math.pi)*math.sin(phi+math.pi)
            goal_y=radius*math.sin(theta+2*math.pi)*math.sin(phi+math.pi)
            goal_z=shift_z + radius*math.cos(phi+math.pi)
                
            # quad="SQ0" + str(id_number) + "s";
            veh="SQ";
            if i <= 9:
                num="0" + str(id_number)
            else:
                num=str(id_number)

            id_number=id_number+1

            if(formation=="square"):
                x=square_starts[i-1][0];
                y=square_starts[i-1][1];
                z=square_starts[i-1][2];

                goal_x=square_goals[i-1][0];
                goal_y=square_goals[i-1][1];
                goal_z=square_goals[i-1][2];

                yaw=square_yaws_deg[i-1]*math.pi/180;
                print("yaw= ", square_yaws_deg[i-1])

            goals[i-1,:] = [goal_x, goal_y, goal_z];

    return goals

def checkGoalReached(num_of_agents):

    goals = getGoals(num_of_agents)

    for i in range(1, num_of_agents+1):
        pos = np.empty(3)
        if i <= 9:
            # print(subprocess.check_output(['rostopic', 'echo', '/SQ0' + str(i) + 's/state/pos/x', '-n', '1']))
            # print(subprocess.check_output(['rostopic', 'echo', '/SQ0' + str(i) + 's/state/pos/x', '-n', '1']).decode().replace("\n---\n",""))1
            # string = subprocess.check_output(['rostopic', 'echo', '/SQ0' + str(i) + 's/state/pos/x', '-n', '1'])
            # print(string.decode())
            pos[0] = float(subprocess.check_output(['rostopic', 'echo', '/SQ0' + str(i) + 's/state/pos/x', '-n', '1']).decode().replace("\n---\n",""))
            pos[1] = float(subprocess.check_output(['rostopic', 'echo', '/SQ0' + str(i) + 's/state/pos/y', '-n', '1']).decode().replace("\n---\n",""))
            pos[2] = float(subprocess.check_output(['rostopic', 'echo', '/SQ0' + str(i) + 's/state/pos/z', '-n', '1']).decode().replace("\n---\n",""))
        else:
            pos[0] = float(subprocess.check_output(['rostopic', 'echo', '/SQ' + str(i) + 's/state/pos/x', '-n', '1']).decode().replace("\n---\n",""))
            pos[1] = float(subprocess.check_output(['rostopic', 'echo', '/SQ' + str(i) + 's/state/pos/y', '-n', '1']).decode().replace("\n---\n",""))
            pos[2] = float(subprocess.check_output(['rostopic', 'echo', '/SQ' + str(i) + 's/state/pos/z', '-n', '1']).decode().replace("\n---\n",""))
        
        print('agent'+str(i))
        print('state'+str(pos))
        print('goal '+str(goals[i-1]))
        goal_radius = 0.15 # set by mader.yaml

        if (LA.norm(goals[i-1,:]-pos) > goal_radius):
            return False

    return True


if __name__ == '__main__':

    # parameters
    num_of_sims=1
    num_of_agents=10

    # initialize commands list
    commands = []

    # other strings
    # folder_bags="/home/kota/data/bags/oldmader";
    folder_bags="/home/kota/data/bags/rmader";
    # folder_txts="/home/kota/data/txt_files/oldmader"
    folder_txts="/home/kota/data/txt_files/rmader"
    # name_node_record="bag_recorder"
    kill_all="tmux kill-server & killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roscore & killall -9 rosmaster & pkill mader_node & pkill -f dynamic_obstacles & pkill -f rosout & pkill -f behavior_selector_node & pkill -f rviz & pkill -f rqt_gui & pkill -f perfect_tracker & pkill -f mader_commands"

    #make sure ROS (and related stuff) is not running
    os.system(kill_all)

    for k in range(num_of_sims):

        if k <= 9:
            sim_id = "0"+str(k)
        else:
            sim_id = str(k)

        commands = []
        name_node_record="bag_recorder"
        commands.append("roscore");

        commands.append("sleep 5.0 && roslaunch mader many_drones.launch action:=controller");
        # commands.append("sleep 1.0 && rosrun mader dynamic_corridor.py");

        commands.append("sleep 3.0 && roslaunch mader many_drones.launch action:=mader sim_id:="+sim_id+" folder:="+folder_txts);
        commands.append("sleep 3.0 && cd "+folder_bags+" && rosbag record -a -o sim_" + sim_id + " __name:="+name_node_record);
        commands.append("sleep 5.0 && roslaunch mader collision_detector.launch num_of_agents:=" + str(num_of_agents));

        #publishing the goal should be the last command
        commands.append("sleep 15.0 && roslaunch mader many_drones.launch action:=send_goal");
        commands.append("sleep 15.0 && tmux detach")

        # print("len(commands)= " , len(commands))
        session_name="run_many_sims_multi_agent_session"
        os.system("tmux kill-session -t" + session_name)
        os.system("tmux new -d -s "+str(session_name)+" -x 300 -y 300")

        # tmux splitting
        for i in range(len(commands)):
            print('splitting ',i)
            os.system('tmux split-window ; tmux select-layout tiled')
       
        time.sleep(5.0)

        for i in range(len(commands)):
            os.system('tmux send-keys -t '+str(session_name)+':0.'+str(i) +' "'+ commands[i]+'" '+' C-m')

        os.system("tmux attach")

        print("Commands sent")

        time.sleep(3.0)

        # check if all the agents reached the goal
        is_goal_reached = False
        tic = time.perf_counter()
        toc = time.perf_counter()

        while (toc - tic < 120 and not is_goal_reached):
            toc = time.perf_counter()
            if(checkGoalReached(num_of_agents)):
                print('all the agents reached the goal')
                is_goal_reached = True

        if (not is_goal_reached):
            os.system('echo "simulation '+sim_id+': not goal reached" >> '+folder_bags+'/status.txt')
        else:
            os.system('echo "simulation '+sim_id+': goal reached" >> '+folder_bags+'/status.txt')


        os.system("rosnode kill "+name_node_record);
        time.sleep(10.0)
        os.system(kill_all)

        time.sleep(10.0)

    time.sleep(3.0)
