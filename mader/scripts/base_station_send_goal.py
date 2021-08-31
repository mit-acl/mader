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

def convertToStringCommand(quad,goal_x,goal_y,goal_z):
    return "rostopic pub /"+quad+"/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: "+str(goal_x)+", y: "+str(goal_y)+", z: "+str(goal_z)+"}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'"

if __name__ == '__main__':

    commands = [];
    num_of_agents = 6;
    sys_arg = "send_global_goals";

    # set quad name and goals (for now, manually)
    
    # agent1 (NUC1)
    quad="SQ01s";
    goal_x = 4.4;
    goal_y = -3.4;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    # agent2 (NUC2)
    quad="SQ02s";
    goal_x = 0.1;
    goal_y = -3.4;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    # agent3 (NUC3)
    quad="SQ03s";
    goal_x = -4.2;
    goal_y = -3.4;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    # agent4 (NUC4)
    quad="SQ04s";
    goal_x = 4.4;
    goal_y = 3.9;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    # agent5 (NUC5)
    quad="SQ05s";
    goal_x = 0.1;
    goal_y = 3.9;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));

    # agent6 (NUC6)
    quad="SQ06s";
    goal_x = -4.2;
    goal_y = 3.9;
    goal_z = 1.8;

    commands.append(convertToStringCommand(quad,goal_x,goal_y,goal_z));
    
    session_name = sys_arg + "_session"
    os.system("tmux kill-session -t" + session_name)
    create_session(session_name, commands) #Kota commented out July 16, 2021
