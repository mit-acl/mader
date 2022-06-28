#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python collision_check.py

#Whenever you want, you can ctrl+C the second command and you will get the absolute and relative velocities

import bagpy
from bagpy import bagreader
import pandas as pd

import rosbag
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import tf
import numpy as np
import matplotlib.pyplot as plt
import os
import glob

if __name__ == '__main__':

    # rosbag name

    # Dont use ~ like this
    cd = "50" # [ms] communication delay
    dc = "100" # [ms] delay check
    source_dir = "/home/kota/data/bags/cd_"+cd+"ms_dc_"+dc+"ms/rmader" # change the source dir accordingly #10 agents
    source_len = len(source_dir)
    # source_dir = "/home/kota/data/bags/multi_agent/sim_num_1_2022-06-24-20-48-34_bag_comm_delay_proof" # change the source dir accordingly #10 agents 
    source_bags = source_dir + "/*.bag" # change the source dir accordingly

    rosbag_list = glob.glob(source_bags)
    rosbag_list.sort() #alphabetically order
    rosbag = []

    for bag in rosbag_list:
        rosbag.append(bag)

    for i in range(len(rosbag)):

        b = bagreader(rosbag[i], verbose=False)
        sim_id = rosbag[i][source_len+5:source_len+7]
        
        log_data = b.message_by_topic("/is_collided")
        if (log_data == None):
            print("sim " + sim_id + ": no collision" )
            os.system("sed -n '"+str(i+1)+"s/$/[[:space:]] no collision/' status.txt > "+source_dir+"/complete_status.txt")
            # os.system('echo "simulation '+sim_id+': no collision" >> '+source_dir+'/collision_status.txt')
        else:
            print("sim " + sim_id + ": ******collision******" )
            os.system("sed -n '"+str(i+1)+"s/$/[[:space:]] no collision/' status.txt > "+source_dir+"/complete_status.txt")
            # os.system('echo "simulation '+sim_id+': ******collision******" > '+source_dir+'/collision_status.txt')



