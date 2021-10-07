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

    os.system("tmux new -d -s "+str(session_name)+" -x 300 -y 300")

    for i in range(len(commands)):
        print('splitting ',i)
        os.system('tmux split-window ; tmux select-layout tiled')
   
    for i in range(len(commands)):
        os.system('tmux send-keys -t '+str(session_name)+':0.'+str(i) +' "'+ commands[i]+'" '+' C-m') 
    print("Commands sent")


#def convertToStringCommand(quad,x,y,z,goal_x,goal_y,goal_z, yaw):
#    return "rostopic pub /"+quad+"/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: "+str(goal_x)+", y: "+str(goal_y)+", z: "+str(goal_z)+"}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'"

def convertToStringCommand(quad,goal_x,goal_y,goal_z):
    return "rostopic pub /"+quad+"/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: "+str(goal_x)+", y: "+str(goal_y)+", z: "+str(goal_z)+"}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'"



if __name__ == '__main__':

    commands = [];
    num_of_agents = 2;
    sys_arg = "send_global_goals";

    # set quad number, initial positions, and goals (for now, manually)
    # agent1 (NUC1)
    quad="HX15";
    goal_x = 4.0;
    goal_y = -3.0;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    # agent2 (NUC2)
    quad="HX16";
    goal_x = 0.0;
    goal_y = -3.0;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    # agent3 (NUC3)
    quad="HX17";
    goal_x = -4.0;
    goal_y = -3.0;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    # agent4 (NUC4)
    quad="HX18";
    goal_x = 4.0;
    goal_y = 3.0;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    # agent5 (NUC5)
    quad="HX19";
    goal_x = 0.0;
    goal_y = 3.0;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    # agent6 (NUC6)
    quad="HX20";
    goal_x = -4.0;
    goal_y = 3.0;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    #x_tmp="{:5.3f}".format(x);
    #y_tmp="{:5.3f}".format(y);
    #z_tmp="{:5.3f}".format(z);

    #goal_x_tmp="{:5.3f}".format(goal_x);
    #goal_y_tmp="{:5.3f}".format(goal_y);
    #goal_z_tmp="{:5.3f}".format(goal_z);
 
    #print (' "start": [',x_tmp,', ',y_tmp,', ',z_tmp,'], "goal": [',goal_x_tmp,', ',goal_y_tmp,', ',goal_z_tmp,']  ')

    #print("len(commands)= " , len(commands))
    session_name = sys_arg + "_session"
    os.system("tmux kill-session -t" + session_name)
    create_session(session_name, commands) #Kota commented out July 16, 2021