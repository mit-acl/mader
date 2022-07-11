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
from mader_msgs.msg import GoalReached

def checkGoalReached(num_of_agents):
    try:
        is_goal_reached = subprocess.check_output(['rostopic', 'echo', '/goal_reached', '-n', '1'], timeout=2).decode()
        print("True")
        return True 
    except:
        print("False")
        return False        

def myhook():
  print("shutdown time!")

if __name__ == '__main__':

    # parameters
    is_oldmader=False
    num_of_sims=60
    num_of_agents=10
    if is_oldmader:
        dc_list = [0, 170, 78, 63, 55, 50_1] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
    else:
        # dc_list = [170, 78, 63, 55, 50_1] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        dc_list = [170] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)

    # folder initialization
    folder_bags_list = []
    folder_txts_list = []

    for dc in dc_list:

        dc_in_ms = dc/1000;

        # mader.yaml modification. comment out delay_check param and is_delaycheck param
        os.system("sed -i '/delay_check/s/^/#/g' $(rospack find mader)/param/mader.yaml")
        os.system("sed -i '/is_delaycheck/s/^/#/g' $(rospack find mader)/param/mader.yaml")

        if is_oldmader:
            folder_bags="/home/kota/data/bags/oldmader/cd50ms"
            folder_txts="/home/kota/data/txt_files/oldmader/cd50ms"
        else:
            folder_bags="/home/kota/data/bags/rmader/cd50msdc"+str(dc)+"ms"
            folder_txts="/home/kota/data/txt_files/rmader/cd50msdc"+str(dc)+"ms"

        # create directy if not exists
        if (not os.path.exists(folder_bags)):
            os.makedirs(folder_bags)

        # create directy if not exists
        if (not os.path.exists(folder_txts)):
            os.makedirs(folder_txts)        

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

            for num in range(1,num_of_agents+1):
                if num <= 9:
                    agent_id = "0"+str(num)
                else:
                    agent_id = str(num)

                commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/mader/delay_check "+str(dc_in_ms))
                if is_oldmader:
                    commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/mader/is_delaycheck false")
                else:
                    commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/mader/is_delaycheck true")

            commands.append("sleep 3.0 && roslaunch mader many_drones.launch action:=controller")
            commands.append("sleep 3.0 && roslaunch mader many_drones.launch action:=mader sim_id:="+sim_id+" folder:="+folder_txts)
            commands.append("sleep 3.0 && cd "+folder_bags+" && rosbag record -a -o sim_" + sim_id + " __name:="+name_node_record)
            commands.append("sleep 3.0 && roslaunch mader collision_detector.launch num_of_agents:=" + str(num_of_agents))
            commands.append("sleep 3.0 && roslaunch mader goal_reached.launch")

            #publishing the goal should be the last command
            commands.append("sleep 10.0 && roslaunch mader many_drones.launch action:=send_goal")
            commands.append("sleep 10.0 && tmux detach")

            # print("len(commands)= " , len(commands))
            session_name="run_many_sims_multi_agent_session"
            os.system("tmux kill-session -t" + session_name)
            os.system("tmux new-session -d -s "+str(session_name)+" -x 300 -y 300")

            # tmux splitting
            for i in range(len(commands)):
                print('splitting ',i)
                os.system('tmux new-window -t ' + str(session_name))
           
            time.sleep(3.0)

            for i in range(len(commands)):
                os.system('tmux send-keys -t '+str(session_name)+':'+str(i) +'.0 "'+ commands[i]+'" '+' C-m')

            os.system("tmux attach")
            print("Commands sent")

            # rospy.init_node('goalReachedCheck_in_sims', anonymous=True)
            # c = GoalReachedCheck_sim()
            # rospy.Subscriber("goal_reached", GoalReached, c.goal_reachedCB)
            # rospy.on_shutdown(myhook)

            # check if all the agents reached the goal
            is_goal_reached = False
            tic = time.perf_counter()
            toc = time.perf_counter()

            while (toc - tic < 60 and not is_goal_reached):
                toc = time.perf_counter()
                if(checkGoalReached(num_of_agents)):
                    print('all the agents reached the goal')
                    is_goal_reached = True
                time.sleep(0.1)

            if (not is_goal_reached):
                os.system('echo "simulation '+sim_id+': not goal reached" >> '+folder_bags+'/status.txt')
            else:
                os.system('echo "simulation '+sim_id+': goal reached" >> '+folder_bags+'/status.txt')

            os.system("rosnode kill "+name_node_record);
            os.system("rosnode kill goalReachedCheck_in_sims")
            os.system("rosnode kill -a")
            time.sleep(1.0)
            os.system(kill_all)
            time.sleep(1.0)

        # uncomment delay_check param
        os.system("sed -i '/delay_check/s/^#//g' $(rospack find mader)/param/mader.yaml")
        os.system("sed -i '/is_delaycheck/s/^#//g' $(rospack find mader)/param/mader.yaml")

        # use old mader only once
        if is_oldmader:
            is_oldmader=False

        time.sleep(3.0)