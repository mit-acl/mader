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

    is_oldmader = True

    if is_oldmader:
        cd_list = [0, 50, 100, 200, 300]
    else:
        cd_list = [50, 100, 100, 200, 300]

    for cd in cd_list:

        is_oldmader=True

        if cd == 0:
            dc_list = [0, 100, 20, 8, 1] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 50:
            dc_list = [0, 120, 56, 51, 50.8, 35, 15] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
            # dc_list = [0, 120] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 100:
            dc_list = [0, 190, 105, 101.3, 101, 75, 25] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
            # dc_list = [0, 170] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 200:
            dc_list = [0, 300]
        elif cd == 300:
            dc_list = [0, 400]

        for dc in dc_list:

            collision_cnt = 0

            if dc == 50.8:
                str_dc = "51_8"
            elif dc == 101.3:
                str_dc = "101_3"
            else:
                str_dc = str(dc)

            if is_oldmader:
                source_dir = "/home/kota/data/bags/oldmader/cd"+str(cd)+"ms" # change the source dir accordingly #10 agents
            else:
                source_dir = "/home/kota/data/bags/rmader/cd"+str(cd)+"ms/dc"+str_dc+"ms" # change the source dir accordingly #10 agents
            
            source_len = len(source_dir)
            source_bags = source_dir + "/*.bag" # change the source dir accordingly

            rosbag_list = glob.glob(source_bags)
            rosbag_list.sort() #alphabetically order
            rosbag = []

            for bag in rosbag_list:
                rosbag.append(bag)

            for i in range(len(rosbag)):

                try:
                    b = bagreader(rosbag[i], verbose=False)
                    sim_id = rosbag[i][source_len+5:source_len+7]
                    log_data = b.message_by_topic("/is_collided")
                    if (log_data == None):
                        print("sim " + sim_id + ": no collision" )
                        os.system('echo "simulation '+sim_id+': no collision" >> '+source_dir+'/collision_status.txt')
                    else:
                        collision_cnt = collision_cnt + 1
                        print("sim " + sim_id + ": ******collision******" )
                        os.system('echo "simulation '+sim_id+': ***collision***" >> '+source_dir+'/collision_status.txt')
                except:
                    pass

            collision_per = 100 - collision_cnt / len(rosbag) * 100
            os.system('paste '+source_dir+'/collision_status.txt '+source_dir+'/status.txt >> '+source_dir+'/complete_status.txt')
            os.system('echo "'+source_dir+'" >> /home/kota/data/collision_count.txt')
            os.system('echo "'+str(collision_cnt)+'/'+str(len(rosbag))+' - '+str(round(collision_per,2))+'%" >> /home/kota/data/collision_count.txt')
            os.system('echo "------------------------------------------------------------" >> /home/kota/data/collision_count.txt')

            is_oldmader = False