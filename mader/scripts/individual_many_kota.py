#!/usr/bin/env python
# coding=utf-8

# /* ----------------------------------------------------------------------------
#  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Jesus Tordesillas, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

#   There's places you need to change depending on each agent
#   Each places are pointed out by #num#, so follow them from 1 to 

import math
import os
import sys
import time
from random import *
# import numpy as np
# from pyquaternion import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def sendCommand(action,quad,x,y,z,psi):
    if(action=="start"):
        #Kota comment: this line launches mader_specific.launch
        os.system("roslaunch mader mader_specific.launch gazebo:=false quad:=" + quad + " x:=" + str(x) + " y:=" + str(y) + " z:=" + str(z) + " psi:=" + str(psi));
    if(action=="mader"):
        #Kota comment: this line launches mader.launch with the argument of quad number
        os.system("roslaunch mader mader.launch quad:="+quad) 
        
if __name__ == '__main__':

    # set quad number, initial positions, and goals (for now, manually)

    # agent#1# (NUC#2#)
    quad="SQ01s"; #3#
    init_x = 4;
    init_y = 0;
    init_z = 1.0;
    init_psi = 0;
    
    sendCommand(sys.argv[1],quad,init_x,init_y,init_z,init_psi);


    x_tmp="{:5.3f}".format(init_x);
    y_tmp="{:5.3f}".format(init_y);
    z_tmp="{:5.3f}".format(init_z);

#    goal_x_tmp="{:5.3f}".format(goal_x);
#    goal_y_tmp="{:5.3f}".format(goal_y);
#    goal_z_tmp="{:5.3f}".format(goal_z);
 
#    print (' "start": [',x_tmp,', ',y_tmp,', ',z_tmp,'], "goal": [',goal_x_tmp,', ',goal_y_tmp,', ',goal_z_tmp,']  ')

    time.sleep(1);