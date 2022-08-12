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

    n_agents = 10

    is_oldmader = True # change here 
    
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

        # this gives you 2d array, row gives you each sims data in corresponding dc
        box_plot_list = [] 

        for dc in dc_list:

            if dc == 50.8:
                str_dc = "51_8"
            elif dc == 101.3:
                str_dc = "101_3"
            else:
                str_dc = str(dc)

            home_dir = "/home/kota/data/bags"

            # source directory
            if is_oldmader:
                source_dir = "/home/kota/data/bags/oldmader/cd"+str(cd)+"ms" # change the source dir accordingly #10 agents
                is_oldmader = False
            else:
                source_dir = "/home/kota/data/bags/rmader/cd"+str(cd)+"ms/dc"+str_dc+"ms" # change the source dir accordingly #10 agents
            
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
                
                # introduced goal_reached topic so no need to check actual_traj

                try:
                    log_data = b.message_by_topic("/goal_reached")
                    log = pd.read_csv(log_data, usecols=["completion_time", "is_goal_reached"])
                    completion_time = log.completion_time.iloc[0]
                    # print('completion time ' + str(completion_time_agent))
                    completion_time_per_sim_list.append(completion_time)
                    box_plot_list.append(completion_time_per_sim_list)

                except:
                    print("agents didn't reach goals")

            os.system('echo "'+source_dir+'" >> /home/kota/data/completion_time.txt')
            os.system('echo "max is '+str(round(max(completion_time_per_sim_list),2))+'s" >> /home/kota/data/completion_time.txt')
            os.system('echo "ave is '+str(round(statistics.mean(completion_time_per_sim_list),2))+'s" >> /home/kota/data/completion_time.txt')
            os.system('echo "------------------------------------------------------------" >> /home/kota/data/completion_time.txt')

                # # get all the agents' actual_traj topic ( this topic publishes as long as it is tranlating, meaning as soon as the agent reaches the goal this topic stop publishing)
                # completion_time_per_agent_list = [] 
                # for j in range(1,n_agents+1):
                #     if j <= 9:
                #         topic_name = "/SQ0"+str(j)+"s/mader/actual_traj"
                #     else:
                #         topic_name = "/SQ"+str(j)+"s/mader/actual_traj"

                #     log_data = b.message_by_topic(topic_name)
                #     log = pd.read_csv(log_data, usecols=["header.stamp.secs", "header.stamp.nsecs"])
                #     log = log.rename(columns={"header.stamp.secs": "secs", "header.stamp.nsecs": "nsecs"})

                #     start_index = 0
                #     while log.secs[start_index] == 0 or log.secs[start_index] == 0:
                #         start_index = start_index + 1

                #     start_time_agent = log.secs[start_index] + log.nsecs[start_index] / 10**9 # [0] is actually 0. You can check that with print(log)
                #     # print('start time ' + str(start_time_agent))
                #     completion_time_agent = log.secs.iloc[-1] + log.nsecs.iloc[-1] / 10**9 - start_time_agent
                #     # print('completion time ' + str(completion_time_agent))
                #     completion_time_per_agent_list.append(completion_time_agent)

                # completion_time_per_sim_list.append(max(completion_time_per_agent_list))

                # # print('sim '+str(sim_id)+': '+str(completion_time_per_sim_list[-1])+' [s]')

            # # save data into csv file
            # dict = {'oldmader': box_plot_list[0], 'rmader 250': box_plot_list[1], 'rmader 87': box_plot_list[2], 'rmader 78': box_plot_list[3], 'rmader 63': box_plot_list[4], 'rmader 55': box_plot_list[5]}  
            # print(len(box_plot_list[0]))
            # print(len(box_plot_list[1]))
            # print(len(box_plot_list[2]))
            # print(len(box_plot_list[3]))
            # print(len(box_plot_list[4]))
            # print(len(box_plot_list[5]))
            # df = pd.DataFrame(dict) 
            # # saving the dataframe 
            # df.to_csv(home_dir+'/completion_time.csv') 

            # plot
            fig = plt.figure()
            # Creating axes instance
            ax = fig.add_axes([0, 0, 1, 1])
            # Creating plot
            bp = ax.boxplot(box_plot_list)
            os.system("mkdir /home/kota/data/images/")        
            plt.savefig('/home/kota/data/images/completion_time.png')
            # show plot
            # plt.show()