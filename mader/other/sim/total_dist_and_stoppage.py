#!/usr/bin/env python  
# Kota Kondo

#  python collision_check.py

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
import statistics

if __name__ == '__main__':

    num_of_agents = 10

    # stop count torelance
    stop_cnt_tol = 1e-3

    is_oldmader = False # change here 
    
    if is_oldmader:
        cd_list = [0, 50, 100, 200, 300]
    else:
        cd_list = [0, 50, 100, 200, 300]

    for cd in cd_list:

        is_oldmader=False

        if cd == 0:
            dc_list = [100, 25, 10, 3] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
            # dc_list = [100] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 50:
            dc_list = [130, 56, 51, 50.8, 35, 15] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
            # dc_list = [130] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 100:
            dc_list = [200, 105, 101.3, 101, 75, 25] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
            # dc_list = [200] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
        elif cd == 200:
            dc_list = [300]
        elif cd == 300:
            dc_list = [400]

        # this gives you 2d array, row gives you each sims data in corresponding dc
        box_plot_list = [] 

        for dc in dc_list:

            if dc == 50.8:
                str_dc = "51_8"
            elif dc == 101.3:
                str_dc = "101_3"
            else:
                str_dc = str(dc)

            # home_dir = "/home/kota/data/bags"
            home_dir = "/media/kota/T7/data"

            # source directory
            if is_oldmader:
                source_dir = home_dir+"/bags/oldmader/cd"+str(cd)+"ms" # change the source dir accordingly #10 agents
                is_oldmader = False
            else:
                source_dir = home_dir+"/bags/rmader/cd"+str(cd)+"ms/dc"+str_dc+"ms" # change the source dir accordingly #10 agents
            
            source_len = len(source_dir)
            source_bags = source_dir + "/*.bag" # change the source dir accordingly
            rosbag_list = glob.glob(source_bags)
            rosbag_list.sort() #alphabetically order
            rosbag = []

            for bag in rosbag_list:
                rosbag.append(bag)

            # read ros bags
            total_dist_list = []
            stop_cnt_list = []
            for i in range(len(rosbag)):

                is_skip_bag = False

                print('rosbag ' + str(rosbag[i]))
                b = bagreader(rosbag[i], verbose=False)
                sim_id = rosbag[i][source_len+5:source_len+7]
                
                # introduced goal_reached topic so no need to check actual_traj

                dist = 0
                stop_cnt = 0
                for i in range(1,num_of_agents+1):
                    
                    stopped = True # in the beginning it is stopped

                    if i < 10:
                        log_data = b.message_by_topic("/SQ0" + str(i) + "s/goal")
                    else:
                        log_data = b.message_by_topic("/SQ" + str(i) + "s/goal")
                
                    log = pd.read_csv(log_data, usecols=["p.x", "p.y", "p.z", "v.x", "v.y", "v.z"])
                    log.columns = ["px", "py", "pz", "vx", "vy", "vz"]
                    
                    ###### difference from the previous pos to the current pos
                    # print(log.diff().to_numpy())
                    diff_matrix = log.diff().to_numpy()

                    # since the first row is NaN, starts 
                    for i in range(1, len(log.diff())):
                        dist += np.linalg.norm(log.diff().to_numpy()[i,0:2])

                    ###### stop count
                    for i in range(len(log.to_numpy())):
                        if np.linalg.norm(log.to_numpy()[i,3:5]) > stop_cnt_tol:
                            stopped = False
                        elif np.linalg.norm(log.to_numpy()[i,3:5]) < stop_cnt_tol and not stopped:
                            stop_cnt = stop_cnt + 1
                            stopped = True

                    stop_cnt = stop_cnt - 1 # for the last stop

                    # in case of collision, stop_cnt won't work, so need to skip the bag
                    if (stop_cnt < 0):
                        is_skip_bag = True
                        print("skip the bag")
                        break

                if not is_skip_bag:
                    dist /= num_of_agents
                    stop_cnt /= num_of_agents
                    total_dist_list.append(dist)
                    stop_cnt_list.append(stop_cnt)

                is_skip_bag = False

            ave_total_dist = sum(total_dist_list)/len(total_dist_list)
            ave_stop_cnt = sum(stop_cnt_list)/len(stop_cnt_list)
                            
            os.system('echo "'+source_dir+'" >> '+home_dir+'/total_dist.txt')
            os.system('echo "ave travel dist '+str(round(ave_total_dist,3))+'m" >> '+home_dir+'/total_dist.txt')
            os.system('echo "------------------------------------------------------------" >> '+home_dir+'/total_dist.txt')

            os.system('echo "'+source_dir+'" >> '+home_dir+'/stop_cnt.txt')
            os.system('echo "ave stop count '+str(round(ave_stop_cnt,3))+'" >> '+home_dir+'/stop_cnt.txt')
            os.system('echo "------------------------------------------------------------" >> '+home_dir+'/stop_cnt.txt')