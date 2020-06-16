#!/usr/bin/env python
# coding=utf-8
# Jesus Tordesillas, jtorde@mit.edu
# date: June 2020

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
    os.system("sed -i '/visual/s/^/#/g' $(rospack find faster)/param/faster.yaml") #comment visual param
    os.system("sed -i '/basis/s/^/#/g' $(rospack find faster)/param/faster.yaml") #comment basis param

    num_of_sims=5;

    commands = []

    folder_bags="/home/jtorde/Desktop/ws/src/faster/results/single_agent";
    all_basis=["MINVO", "BEZIER", "B_SPLINE"] #or"MINVO", "BEZIER", "B_SPLINE"
    name_node_record="bag_recorder"
    kill_all="tmux kill-server & killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roscore & killall -9 rosmaster & pkill faster_node & pkill -f dynamic_obstacles & pkill -f rosout & pkill -f behavior_selector_node & pkill -f rviz & pkill -f rqt_gui & pkill -f perfect_tracker & pkill -f faster_commands"

    #make sure ROS (and related stuff) is not running
    os.system(kill_all)


    
    for j in range(len(all_basis)):
        basis=all_basis[j]

        for i in range(num_of_sims):

            commands = []

            commands.append("roslaunch faster all.launch gui:=false rviz:=false");
            commands.append("sleep 1.5 && rosparam set /SQ01s/faster/basis "+basis); #Remember to comment the parameter "basis" in faster.yaml before running this file
            commands.append("sleep 1.5 && rosparam set /SQ01s/faster/visual false"); #Remember to comment the parameter "visual" in faster.yaml before running this file

            commands.append("sleep 4.0 && roslaunch faster faster.launch");
            commands.append("sleep 4.0 && cd "+folder_bags+" && rosbag record -o "+basis+"_"+str(i)+" /SQ01s/goal __name:="+name_node_record);
            #publishing the goal should be the last command
            commands.append("sleep 4.0 && rostopic pub /SQ01s/term_goal geometry_msgs/PoseStamped \'{header: {stamp: now, frame_id: \"world\"}, pose: {position: {x: 75, y: 0, z: 1}, orientation: {w: 1.0}}}\'");

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

            time.sleep(3.0)
            pos_string=""
            while (pos_string.find('74.')==-1 and pos_string.find('75.')==-1): #note the point after the 49, to make sure they are the first two numbers, and not the decimals
                try:
                    pos_string =str(subprocess.check_output(['rostopic', 'echo', '/SQ01s/state/pos/x', '-n', '1']))
                    print("Currently at ",pos_string)
                except:
                    print("An Error occurred")


            print("Currently at ",pos_string)
            print("Goal is reached, killing the bag node")
            os.system("rosnode kill "+name_node_record);
            time.sleep(0.5)
            print("Killing the rest")
            os.system(kill_all)

    time.sleep(3.0)
    os.system("sed -i '/visual/s/^#//g' $(rospack find faster)/param/faster.yaml") #comment out visual param
    os.system("sed -i '/basis/s/^#//g' $(rospack find faster)/param/faster.yaml") #comment out basis param