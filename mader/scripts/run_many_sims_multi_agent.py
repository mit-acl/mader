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
from mader_msgs.msg import CompletionTime

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
    is_oldmader=True
    num_of_sims=1
    num_of_agents=10
    how_long_to_wait = 30 #[s]
    if is_oldmader:
        cd_list = [50, 100, 200, 300]
    else:
        cd_list = [50, 100]
        
    # folder initialization
    folder_bags_list = []
    folder_txts_list = []

    for cd in cd_list:

        is_oldmader=True

        if cd == 50:
            dc_list = [0, 160, 55, 51, 50.5, 50.1] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
            # dc_list = [0, 160] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 100:
            dc_list = [0, 210, 105, 101, 100.5, 100.1] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
            # dc_list = [0, 210] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        else:
            dc_list =[0]

        for dc in dc_list:

            dc_in_ms = dc/1000;
            cd_in_ms = cd/1000;

            if dc == 50.5:
                str_dc = "50_5"
            elif dc == 50.1:
                str_dc = "50_1"
            elif dc == 100.5:
                str_dc = "100_5"
            elif dc == 100.1:
                str_dc = "100_1"

            # mader.yaml modification. comment out delay_check param and is_delaycheck param
            os.system("sed -i '/delay_check/s/^/#/g' $(rospack find mader)/param/mader.yaml")
            os.system("sed -i '/is_delaycheck/s/^/#/g' $(rospack find mader)/param/mader.yaml")
            os.system("sed -i '/simulated_comm_delay/s/^/#/g' $(rospack find mader)/param/mader.yaml")

            if is_oldmader:
                folder_bags="/home/kota/data/bags/oldmader/cd"+str(cd)+"ms"
                folder_txts="/home/kota/data/txt_files/oldmader/cd"+str(cd)+"ms"
                folder_csv="/home/kota/data/csv/oldmader/cd"+str(cd)+"ms"
            else:
                folder_bags="/home/kota/data/bags/rmader/cd"+str(cd)+"ms/dc"+str_dc+"ms"
                folder_txts="/home/kota/data/txt_files/rmader/cd"+str(cd)+"ms/dc"+str_dc+"ms"
                folder_csv="/home/kota/data/csv/rmader/cd"+str(cd)+"ms/dc"+str_dc+"ms"

            # create directy if not exists
            if (not os.path.exists(folder_bags)):
                os.makedirs(folder_bags)

            # create directy if not exists
            if (not os.path.exists(folder_txts)):
                os.makedirs(folder_txts)

             # create directy if not exists
            if (not os.path.exists(folder_csv)):
                os.makedirs(folder_csv)        

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
                commands.append("roscore")

                for num in range(1,num_of_agents+1):
                    if num <= 9:
                        agent_id = "0"+str(num)
                    else:
                        agent_id = str(num)

                    commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/mader/delay_check "+str(dc_in_ms))
                    commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/mader/simulated_comm_delay "+str(cd_in_ms))
                    if is_oldmader:
                        commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/mader/is_delaycheck false")
                    else:
                        commands.append("sleep 3.0 && rosparam set /SQ"+agent_id+"s/mader/is_delaycheck true")

                commands.append("sleep 3.0 && roslaunch mader many_drones.launch action:=controller")
                commands.append("sleep 3.0 && roslaunch mader many_drones.launch action:=mader sim_id:="+sim_id+" folder:="+folder_txts)
                commands.append("sleep 3.0 && cd "+folder_bags+" && rosbag record -a -o sim_" + sim_id + " __name:="+name_node_record)
                commands.append("sleep 3.0 && roslaunch mader collision_detector.launch num_of_agents:=" + str(num_of_agents))
                commands.append("sleep 3.0 && roslaunch mader ave_distance.launch num_of_agents:="+str(num_of_agents)+" folder_loc:="+folder_csv+" sim:="+sim_id)

                #publishing the goal should be the last command
                commands.append("sleep 10.0 && roslaunch mader many_drones.launch action:=send_goal")
                commands.append("sleep 10.0 && roslaunch mader goal_reached.launch") #we are calculating completion time here so sleep time needs to be the same as send_goal
                commands.append("sleep 12.0 && tmux detach")

                # print("len(commands)= " , len(commands))
                session_name="run_many_sims_multi_agent_session"
                os.system("tmux kill-session -t" + session_name)
                os.system("tmux new-session -d -s "+str(session_name)+" -x 300 -y 300")

                # tmux splitting
                for i in range(len(commands)):
                    # print('splitting ',i)
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
                os.system('tmux new-window -t ' + str(session_name))

                while (toc - tic < how_long_to_wait and not is_goal_reached):
                    toc = time.perf_counter()
                    if(checkGoalReached(num_of_agents)):
                        print('all the agents reached the goal')
                        time.sleep(2) # gives us time to write csv file for ave distance
                        is_goal_reached = True
                    time.sleep(0.1)

                if (not is_goal_reached):
                    os.system('echo "simulation '+sim_id+': not goal reached" >> '+folder_bags+'/status.txt')
                else:
                    os.system('echo "simulation '+sim_id+': goal reached" >> '+folder_bags+'/status.txt')

                os.system("rosnode kill "+name_node_record);
                # os.system("rosnode kill goalReachedCheck_in_sims")
                # os.system("rosnode kill -a")
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


    # After the simulations 
    commands = []
    commands.append("sleep 3.0 && roscd mader && cd other/sim && python collision_check.py")
    commands.append("sleep 3.0 && roscd mader && cd other/sim && python completion_time.py")
    commands.append("sleep 3.0 && roscd mader && cd other/sim && python comm_delay_histogram_percentile.py")
    commands.append("sleep 3.0 && roscd mader && cd other/sim && python ave_distance_csv2txt.py")
    commands.append("sleep 3.0 && roscd mader && cd other/sim && python missed_msgs_count.py")

    # tmux splitting
    for i in range(len(commands)):
        # print('splitting ',i)
        os.system('tmux new-window -t ' + str(session_name))
   
    time.sleep(3.0)

    for i in range(len(commands)):
        os.system('tmux send-keys -t '+str(session_name)+':'+str(i) +'.0 "'+ commands[i]+'" '+' C-m')

    os.system("tmux attach")
    print("Commands sent")