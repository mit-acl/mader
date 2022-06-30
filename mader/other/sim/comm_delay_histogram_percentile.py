#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python comm_delay_histogram_percentile.py input_comm_delay
#  Ex. python comm_delay_histogram_percentile.py 0.150

# change cd and dc

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
import sys
import scipy
import numpy

if __name__ == '__main__':

    # Dont use ~ like this
    cd = "50" # [ms] communication delay
    dc = "250" # [ms] delay check
    figname = 'cd_'+cd+'_dc_'+dc+'_rmader_comm_delay_histogram.png'
    source_dir = "/home/kota/data/bags" # change the source dir accordingly #10 agents 
    # source_dir = "/home/kota/data/bags/multi_agent/sim_num_1_2022-06-24-20-48-34_bag_comm_delay_proof" # change the source dir accordingly #10 agents 
    source_bags = source_dir + "/cd_"+cd+"ms_dc_"+dc+"ms/rmader/*.bag" # change the source dir accordingly

    rosbag_list = glob.glob(source_bags)
    rosbag_list.sort() #alphabetically order
    rosbag = []
    comm_delay = []
    num_of_agents = 10

    # comm_delay you use
    input_comm_delay = float(sys.argv[1])

    for bag in rosbag_list:
        rosbag.append(bag)

    for i in range(len(rosbag)):

        b = bagreader(rosbag[i], verbose=False);
        
        for i in range(1,num_of_agents+1):
            if i < 10:
                log_data = b.message_by_topic("/SQ0" + str(i) + "s/mader/comm_delay")
            else:
                log_data = b.message_by_topic("/SQ" + str(i) + "s/mader/comm_delay")

            log = pd.read_csv(log_data)

            for j in range(len(log.comm_delay)):
                comm_delay.append(log.comm_delay[j])

    # print percentile
    # comm_delay_arr = numpy.array(comm_delay)
    # percentile = scipy.stats.percentileofscore(comm_delay_arr, input_comm_delay, kind='mean')
    # os.system('echo "cd='+cd+', dc='+dc+':   '+str(input_comm_delay) + ' is ' + str(percentile) + '-th percentile" >> '+source_dir+'/comm_delay_percentile.txt')
    
    # in case you wanna calculate the value of q-th percentile
    q = 50
    print(str(q) + "-th percentile value is " + str(numpy.percentile(comm_delay_arr, q)))

    # # print(comm_delay)
    # max_comm_delay = max(comm_delay)

    # n, bins, patches = plt.hist(x=comm_delay)
    # plt.grid(axis='y', color='black', alpha=0.2)
    # plt.title('Comm delay histogram \n max comm_delay is '+str(round(max_comm_delay,3))+' [s]')
    # plt.xlabel("comm delay [s]")
    # plt.ylabel("count")
    # plt.savefig('data/'+figname)
    # # plt.show()
