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

if __name__ == '__main__':

    n_agents = 10

    # rosbag name

    # Dont use ~ like this
    cd = "50" # [ms] communication delay

    is_oldmader = True # change here 
    
    if is_oldmader:
        dc_list = [0, 250, 87, 78, 63, 55] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
    else:
        dc_list = [250, 87, 78, 63, 55] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)

    # this gives you 2d array, row gives you each sims data in corresponding dc
    box_plot_list = [] 

    for dc in dc_list:

        home_dir = "/home/kota/data/bags"

        # source directory
        if is_oldmader:
            source_dir = "/home/kota/data/bags/oldmader/cd"+cd+"ms" # change the source dir accordingly #10 agents
        else:
            source_dir = "/home/kota/data/bags/rmader/cd"+cd+"msdc"+str(dc)+"ms" # change the source dir accordingly #10 agents
        
        source_len = len(source_dir)
        source_bags = source_dir + "/*.bag" # change the source dir accordingly
        rosbag_list = glob.glob(source_bags)
        rosbag_list.sort() #alphabetically order
        rosbag = []

        for bag in rosbag_list:
            rosbag.append(bag)

        # read ros bags
        completion_time_per_sim_list = []
        completion_time_per_sim = 0.0
        for i in range(len(rosbag)):
            print('rosbag ' + str(rosbag[i]))
            b = bagreader(rosbag[i], verbose=False)
            sim_id = rosbag[i][source_len+5:source_len+7]
            
            # get all the agents' actual_traj topic ( this topic publishes as long as it is tranlating, meaning as soon as the agent reaches the goal this topic stop publishing)
            completion_time_per_agent_list = [] 
            for j in range(1,n_agents+1):
                if j <= 9:
                    topic_name = "/SQ0"+str(j)+"s/mader/actual_traj"
                else:
                    topic_name = "/SQ"+str(j)+"s/mader/actual_traj"

                log_data = b.message_by_topic(topic_name)
                log = pd.read_csv(log_data, usecols=["header.stamp.secs", "header.stamp.nsecs"])
                log = log.rename(columns={"header.stamp.secs": "secs", "header.stamp.nsecs": "nsecs"})

                start_index = 0
                while log.secs[start_index] == 0 or log.secs[start_index] == 0:
                    start_index = start_index + 1

                start_time_agent = log.secs[start_index] + log.nsecs[start_index] / 10**9 # [0] is actually 0. You can check that with print(log)
                print('start time ' + str(start_time_agent))
                completion_time_agent = log.secs.iloc[-1] + log.nsecs.iloc[-1] / 10**9 - start_time_agent
                print('completion time ' + str(completion_time_agent))
                completion_time_per_agent_list.append(completion_time_agent)

            completion_time_per_sim_list.append(max(completion_time_per_agent_list))

            print('sim '+sim_id+': '+completion_time_per_sim_list[-1]+' [s]')

        box_plot_list.append(completion_time_per_sim_list)

        is_oldmader = False

    # save data into csv file
    dict = {'oldmader': box_plot_list[0], 'rmader 250': box_plot_list[1], 'rmader 87': box_plot_list[2], 'rmader 78': box_plot_list[3], 'rmader 63': box_plot_list[4], 'rmader 55': box_plot_list[5]}  
    df = pd.DataFrame(dict) 
    # saving the dataframe 
    df.to_csv('completion_time.csv') 

    # plot
    fig = plt.figure()
    # Creating axes instance
    ax = fig.add_axes([0, 0, 1, 1])
    # Creating plot
    bp = ax.boxplot(box_plot_list)
    plt.savefig(home_dir+'completion_time.png')
    # show plot
    # plt.show()