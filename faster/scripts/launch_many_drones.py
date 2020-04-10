#!/usr/bin/env python
# coding=utf-8
# author: thinkycx
# date: 2018-01-11
# modified by: Jesus Tordesillas, jtorde@mit.edu
# date: 2020-February



import math
import os
import sys
import time
from random import *
# import numpy as np
# from pyquaternion import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def create_session(session_name, commands):

    os.system("tmux new -d -s "+str(session_name)+" -x 300 -y 300")

    for i in range(len(commands)):
        print('splitting ',i)
        os.system('tmux split-window ; tmux select-layout tiled')
   
    for i in range(len(commands)):
        os.system('tmux send-keys -t '+str(session_name)+':0.'+str(i) +' "'+ commands[i]+'" '+' C-m')
    print("Commands sent")


if __name__ == '__main__':
    commands = []
    num_of_agents=4; #even number
    radius=4.5;
    half_of_agents=num_of_agents/2.0
    dist_bet_groups=6.0
    random_01=randint(0, 1)
    print("random_01= ", random_01)

    positions=[];
    angles=[];

    z_value=0.0;

    for i in range(1,num_of_agents+1):
        theta=(2*math.pi)*i/(1.0*num_of_agents)
        angles.append(theta)

    for i in range(1,num_of_agents+1):

        # group = (i>half_of_agents)

        # x= i if group == 0 else (i-half_of_agents)
        # y= dist_bet_groups*group   
        # z=0

        # goal_x=half_of_agents-x
        # goal_y=random_01*(dist_bet_groups-y) + (1-random_01)*y 
        # goal_z=0

        theta=angles[i-1];

        x=radius*math.cos(theta)
        y=radius*math.sin(theta)
        z=z_value
        z_value=1.0 #From now on, stay on z=1 meter

        pitch=0.0;
        roll=0.0;
        yaw= theta+math.pi  

        theta=theta+math.pi

        goal_x=radius*math.cos(theta)
        goal_y=radius*math.sin(theta)
        goal_z=z

        angles[i-1]=theta;

      
        # quat = quaternion_from_euler(yaw, pitch, roll, 'szyx')
        # print (quat)


        quad="SQ0" + str(i) + "s";
        print ("quad= ",quad)
        print ("goal_y= ",goal_y)
        print ("goal_x= ",goal_x)
        if(sys.argv[1]=="start"):
            commands.append("roslaunch faster faster_specific.launch gazebo:=false quad:="+quad+" x:="+str(x)+" y:="+str(y)+" z:="+str(z)+" yaw:="+str(yaw))
        if(sys.argv[1]=="send_goal"):
            commands.append("rostopic pub /"+quad+"/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: "+str(goal_x)+", y: "+str(goal_y)+", z: "+str(goal_z)+"}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'")
        if(sys.argv[1]=="faster"):
            commands.append("roslaunch faster faster.launch quad:="+quad)
    print("len(commands)= " , len(commands))
    session_name=sys.argv[1] + "_session"
    os.system("tmux kill-session -t" + session_name)
    create_session(session_name, commands)
    if(sys.argv[1]!="send_goal"):
        os.system("tmux attach") #comment if you don't want to visualize all the terminals
    else: ##if send_goal, kill after some time
        time.sleep(num_of_agents); #The more agents, the more I've to wait to make sure the goal is sent correctly
        os.system("tmux kill-session -t" + session_name)





#import libtmux
    # panes=win.list_panes()
    # print("panes.size=", len(panes))
    # for i in range(len(panes)):
    #     panes[i].send_keys(commands[i])


    # logging.info(panes)
    # logging.info(win)

            #win.select_layout(layout='tiled')
        #win.split_window()
        #win.cmd('select-layout tiled')  
        #win.select_layout(layout='tiled')
        #win.cmd('split-window', '-h')    

                #win.cmd('select-layout tiled')
        #os.system("tmux attach")
        #os.system("tmux select-layout tiled")

            #os.system("tmux attach")

    # win = session.new_window(attach=False, window_name="win")
    # win.select_layout(layout='tiled')
    #logging.info(commands)
    # pane_NUM = 3
    # WINDOW_NUM = int(math.ceil(len(commands)/4.0))  # in python3, we can use 4 also

    # server = libtmux.Server()
    # session = server.new_session(session_name)
    # panes = []