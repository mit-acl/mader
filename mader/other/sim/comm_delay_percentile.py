#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python comm_delay_percentile.py input_comm_delay
#  Ex. python comm_delay_percentile.py 0.150

# input: comm delay (eg 150ms) that you want to know in which percentile does it fit in

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

    # rosbag name

    # Dont use ~ like this
    # source_file = "~/Research/data/bags/mader/multi_agent/2022-06-23-20-02-26.bag" # change the source dir accordingly
    source_file = "/home/kota/Research/data/bags/mader/multi_agent/sim_00_2022-06-26-14-17-29.bag" # change the source dir accordingly #10 agents 

    # comm_delay you use
    input_comm_delay = float(sys.argv[1])

    rosbag_list = glob.glob(source_file)
    rosbag = []
    comm_delay = []
    num_of_agents = 10

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
        comm_delay_arr = numpy.array(comm_delay)
        percentile = scipy.stats.percentileofscore(comm_delay_arr, input_comm_delay, kind='mean')
        print(str(input_comm_delay) + ' is ' + str(percentile) + '-th percentile')