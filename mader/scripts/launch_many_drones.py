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
from random import *
# import numpy as np
# from pyquaternion import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def create_session(session_name, commands):

    os.system("tmux new-session -d -s "+str(session_name)+" -x 300 -y 300")

    for i in range(len(commands)):
        print('splitting ',i)
        os.system('tmux split-window ; tmux select-layout tiled')
   
    for i in range(len(commands)):
        # os.system('tmux send-keys -t '+str(session_name)+':1.'+str(i+1) +' "'+ commands[i]+'" '+' C-m')
        os.system('tmux send-keys -t '+str(session_name)+':0.'+str(i+1) +' "'+ commands[i]+'" '+' C-m')
    
    # os.system('tmux send-keys -t '+str(session_name)+':1.'+str(i+1) +' "tmux kill-server" ')
    os.system('tmux send-keys -t '+str(session_name)+':0.'+str(i+1) +' "tmux kill-server" ')

    print("Commands sent")


def convertToStringCommand(action,veh,num,x,y,z,goal_x,goal_y,goal_z, yaw):
    # if(action=="base_station"):
    #     return "roslaunch mader base_station.launch type_of_environment:=dynamic_forest";
    if(action=="controller"):
        quad = veh + num + "s"
        return "roslaunch mader perfect_tracker_and_sim.launch quad:=" + quad + " x:=" + str(x) + " y:=" + str(y)
    if(action=="send_goal"):
        quad = veh + num + "s";
        return "rostopic pub /"+quad+"/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: "+str(goal_x)+", y: "+str(goal_y)+", z: "+str(goal_z)+"}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'"
    if(action=="mader"):
        return "roslaunch mader onboard.launch veh:="+veh+" num:="+num+" 2>&1 | tee ~/data/txt_files/"+veh+num+"_mader_$(date '+%Y_%m_%d_%H_%M_%S').txt"
        # return "roslaunch mader onboard.launch veh:="+veh+" num:="+num #+ " >> "+quad+".txt" #Kota comment: this line launches mader.launch with the argument of quad number
        # return "script -q -c 'roslaunch mader mader.launch quad:="+quad + "' "+quad+".txt"
        

if __name__ == '__main__':
    # formation="sphere", "square" "circle"
    formation="circle"
    commands = []
    num_of_agents=10; 
    radius=10;


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

    print("num_mer= ", num_mer)
    print("num_of_agents_per_mer= ", num_of_agents_per_mer)

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


            commands.append(convertToStringCommand(sys.argv[1],veh,num,x,y,z,goal_x,goal_y,goal_z, yaw));

            x_tmp="{:5.3f}".format(x);
            y_tmp="{:5.3f}".format(y);
            z_tmp="{:5.3f}".format(z);

            goal_x_tmp="{:5.3f}".format(goal_x);
            goal_y_tmp="{:5.3f}".format(goal_y);
            goal_z_tmp="{:5.3f}".format(goal_z);
 
            print (' "start": [',x_tmp,', ',y_tmp,', ',z_tmp,'], "goal": [',goal_x_tmp,', ',goal_y_tmp,', ',goal_z_tmp,']  ')


    print("len(commands)= " , len(commands))
    session_name=sys.argv[1] + "_session"
    os.system("tmux kill-session -t" + session_name)
    create_session(session_name, commands) #Kota commented out July 16, 2021
    if(sys.argv[1]!="send_goal"):
        os.system("tmux attach") #comment if you don't want to visualize all the terminals
        time.sleep(1); #Kota added to make this "if statement" works even when i comment out the above line
    else: ##if send_goal, kill after some time
        time.sleep(num_of_agents); #The more agents, the more I've to wait to make sure the goal is sent correctly
        os.system("tmux kill-session -t" + session_name)

    # half_of_agents=num_of_agents/2.0
    # dist_bet_groups=6.0
    # random_01=randint(0, 1)
    # print("random_01= ", random_01)

    # positions=[];
    # thetas=[];

    # z_value=0.0;


    # for i in range(1,num_of_agents+1):
    #     theta=(2*math.pi)*i/(1.0*num_of_agents)
    #     thetas.append(theta)

    # for i in range(1,num_of_agents+1):

    #     # group = (i>half_of_agents)

    #     # x= i if group == 0 else (i-half_of_agents)
    #     # y= dist_bet_groups*group   
    #     # z=0

    #     # goal_x=half_of_agents-x
    #     # goal_y=random_01*(dist_bet_groups-y) + (1-random_01)*y 
    #     # goal_z=0

    #     if(sphere):
    #         x=radius*math.cos(theta)*math.sin(phi)
    #         y=radius*math.sin(theta)*math.sin(phi)
    #         z=radius*cos(phi)

    #     theta=thetas[i-1];

    #     x=radius*math.cos(theta)
    #     y=radius*math.sin(theta)
    #     z=1.0 #z_value
    #     #z_value=1.0 #From now on, stay on z=1 meter

    #     pitch=0.0;
    #     roll=0.0;
    #     yaw= theta+math.pi  

    #     theta=theta+math.pi

    #     goal_x=radius*math.cos(theta)
    #     goal_y=radius*math.sin(theta)
    #     goal_z=z

    #     thetas[i-1]=theta;

      
    #     # quat = quaternion_from_euler(yaw, pitch, roll, 'szyx')
    #     # print (quat)


    #     quad="SQ0" + str(i) + "s";
    #     print ("quad= ",quad)
    #     print ("goal_y= ",goal_y)
    #     print ("goal_x= ",goal_x)
    #     if(sys.argv[1]=="start"):
    #         commands.append("roslaunch mader mader_specific.launch gazebo:=false quad:="+quad+" x:="+str(x)+" y:="+str(y)+" z:="+str(z)+" yaw:="+str(yaw))
    #     if(sys.argv[1]=="send_goal"):
    #         commands.append("rostopic pub /"+quad+"/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: "+str(goal_x)+", y: "+str(goal_y)+", z: "+str(goal_z)+"}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'")
    #     if(sys.argv[1]=="mader"):
    #         commands.append("roslaunch mader mader.launch quad:="+quad)






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
