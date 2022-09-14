#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python missed_msgs_count.py

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

    is_oldmader = False # change here

    n_agents = 10

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

        for dc in dc_list:


            dc_in_s = dc/1000;
            cd_in_s = cd/1000;

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

            # going through the rosbags
            ave_list = [] # size will be num of sims
            for i in range(len(rosbag)):
                b = bagreader(rosbag[i], verbose=False)
                sim_id = rosbag[i][source_len+5:source_len+7]

                # going through the agents
                missed_msgs_list_per_sim = [] # size will be num_of_agents
                msgs_list_per_sim = [] # size will be num_of_agents
                
                # using missed_msgs_cnt has a racing problem 
                # for j in range(1,n_agents+1):
                #     if j <= 9:
                #         topic_name = "/SQ0"+str(j)+"s/mader/missed_msgs_cnt"
                #     else:
                #         topic_name = "/SQ"+str(j)+"s/mader/missed_msgs_cnt"
                
                #     log_data = b.message_by_topic(topic_name)

                #     if (log_data == None):
                #         pass
                #     else:
                #         log = pd.read_csv(log_data)
                #         missed_msgs_list_per_sim.append(log.missed_msgs_cnt[0])
                #         msgs_list_per_sim.append(log.msgs_cnt[0])
                #         # print(log.missed_msgs_cnt[0])
                
                msgs_cnt = 0
                missed_msgs_cnt = 0
                ave = 0

                for i in range(1,n_agents+1):
                    if i < 10:
                        log_data = b.message_by_topic("/SQ0" + str(i) + "s/mader/comm_delay")
                    else:
                        log_data = b.message_by_topic("/SQ" + str(i) + "s/mader/comm_delay")

                    # print(log_data)
                    try:
                        log = pd.read_csv(log_data)

                        for j in range(len(log.comm_delay)):
                            msgs_cnt = msgs_cnt + 1
                            # print(log.comm_delay[j])
                            if log.comm_delay[j] > dc_in_s:
                                missed_msgs_cnt = missed_msgs_cnt + 1
                    except:
                        pass

                try:
                    ave = missed_msgs_cnt/msgs_cnt
                    ave_list.append(ave)
                    # os.system('echo "simulation '+sim_id+': missed_msgs_cnt'+ave_missed_msgs_cnt+'" >> '+source_dir+'/missed_msgs_cnt.txt')
                except:
                    pass

            try:      
                ave_missed_per_dc = sum(ave_list)/len(ave_list)
                os.system('echo "'+source_dir+'" >> /home/kota/data/missed_msgs_cnt.txt')
                os.system('echo " missed/total '+str(round(ave_missed_per_dc*100,2))+'%" >> /home/kota/data/missed_msgs_cnt.txt')
                os.system('echo "------------------------------------------------------------" >> /home/kota/data/missed_msgs_cnt.txt')
            except:
                pass
            
            is_oldmader = False