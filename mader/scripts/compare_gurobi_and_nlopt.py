
#!/usr/bin/env python
# coding=utf-8

# /* ----------------------------------------------------------------------------
#  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Jesus Tordesillas, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import math
import os
import sys
import time
import rospy
from snapstack_msgs.msg import State
import subprocess

if __name__ == '__main__':

    #Let's start by commenting the visual and basis params (we will set them in this file)

    #https://stackoverflow.com/questions/24889346/how-to-uncomment-a-line-that-contains-a-specific-string-using-sed/24889374
    os.system("sed -i '/visual/s/^/#/g' $(rospack find mader)/param/mader.yaml") #comment visual param
    os.system("sed -i '/basis/s/^/#/g' $(rospack find mader)/param/mader.yaml") #comment basis param

    num_of_sims=1;
    total_num_of_obs=[250]#[1000]#[50,400,500,600,700]#[150, 200, 250, 300, 350] #[340,380,420,460,500]; #140,180,220,260,300
    commands = []

    folder_bags="/home/kota/data/gurobi_nlopt/bags" # change folder name if necessary

    os.system("mkdir -p /home/kota/data/gurobi_nlopt/bags")
    all_basis=["MINVO"] #or"MINVO", "BEZIER", "B_SPLINE"
    name_node_record="bag_recorder"
    kill_all="tmux kill-server & killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roscore & killall -9 rosmaster & pkill mader_node & pkill -f dynamic_obstacles & pkill -f rosout & pkill -f behavior_selector_node & pkill -f rviz & pkill -f rqt_gui & pkill -f perfect_tracker & pkill -f mader_commands"

    #make sure ROS (and related stuff) is not running
    os.system(kill_all)


    for k in range(len(total_num_of_obs)):
        for j in range(len(all_basis)):
            basis=all_basis[j]

            for s in range(num_of_sims):

                commands = []

                commands.append("roslaunch mader mader_specific_and_general.launch gui:=false rviz:=false environment:=false");
                commands.append("sleep 1.0 &&rosrun mader dynamic_corridor.py "+str(total_num_of_obs[k]));
                commands.append("sleep 3.0 && rosparam set /SQ01s/mader/basis "+basis); #Remember to comment the parameter "basis" in mader.yaml before running this file
                commands.append("sleep 3.0 && rosparam set /SQ01s/mader/visual false"); #Remember to comment the parameter "visual" in mader.yaml before running this file
                # commands.append("sleep 3.0 && rosrun rviz rviz -d  $(rospack find mader)/rviz_cfgs/mader.rviz")

                commands.append("sleep 5.0 && roslaunch mader mader.launch");
                commands.append("sleep 5.0 && cd "+folder_bags+" && rosbag record -o "+basis+"_obs_"+str(total_num_of_obs[k])+"_sim_"+str(s)+" /SQ01s/goal /SQ01s/term_goal __name:="+name_node_record);
                #publishing the goal should be the last command
                commands.append("sleep 15.0 && rostopic pub /SQ01s/term_goal geometry_msgs/PoseStamped \'{header: {stamp: now, frame_id: \"world\"}, pose: {position: {x: 75, y: 0, z: 1}, orientation: {w: 1.0}}}\'");

                print("len(commands)= " , len(commands))
                session_name="run_many_sims_single_agent_session"
                os.system("tmux kill-session -t" + session_name)

                os.system("tmux new -d -s "+str(session_name)+" -x 300 -y 300")

                for i in range(len(commands)):
                    print('splitting ',i)
                    os.system('tmux split-window ; tmux select-layout tiled')
               
                for i in range(len(commands)):
                    os.system('tmux send-keys -t '+str(session_name)+':0.'+str(i) +' "'+ commands[i]+'" '+' C-m')

                print("Commands sent")

                # kill after how_long
                how_long = 60 #[s]
                tic = time.perf_counter()
                goal_reached = True

                time.sleep(3.0)
                pos_string=""
                while (pos_string.find('74.')==-1 and pos_string.find('75.')==-1): #note the point after the 49, to make sure they are the first two numbers, and not the decimals
                    toc = time.perf_counter()
                    if toc - tic > how_long:
                        goal_reached = False
                        break
                    try:
                        pos_string =str(subprocess.check_output(['rostopic', 'echo', '/SQ01s/state/pos/x', '-n', '1']))
                        print("Currently at ",pos_string, "[Sim "+str(s)+", with basis="+basis+", with total_num_of_obs="+str(total_num_of_obs[k])+"]");
                    except:
                        print("An Error occurred")

                if goal_reached:
                    print("Currently at ",pos_string)
                    print("Goal is reached, killing the bag node")
                    os.system('echo "sim'+str(s)+' goal reached" >> /home/kota/data/gurobi_nlopt/bags/status.txt')
                else:
                    print("Time out")
                    os.system('echo "sim'+str(s)+' not goal reached" >> /home/kota/data/gurobi_nlopt/bags/status.txt')

                os.system("rosnode kill "+name_node_record);
                time.sleep(0.5)
                print("Killing the rest")
                os.system(kill_all)

    time.sleep(3.0)
    os.system("sed -i '/visual/s/^#//g' $(rospack find mader)/param/mader.yaml") #comment out visual param
    os.system("sed -i '/basis/s/^#//g' $(rospack find mader)/param/mader.yaml") #comment out basis param

    os.system('python $(rospack find mader)/scripts/statistics.py "/home/kota/data/gurobi_nlopt/bags/*.bag"')
