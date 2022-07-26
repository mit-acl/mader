#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python comm_delay_histogram_percentile.py
#  Ex. python comm_delay_histogram_percentile.py

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

    cd = "50" # [ms] communication delay


    # you wanna get histogram or know the value at q-th percentile
    is_histogram = True
    # q-th percentile
    q = 75

    is_oldmader = True
    num_of_agents = 10

    if is_oldmader:
        dc_list = [0, 200, 170, 120, 100, 78, 63, 55, 51] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)
    else:
        dc_list = [200, 170, 120, 100, 78, 63, 55, 51] #dc_list[0] will be used for old mader (which doesn't need delay check) so enter some value (default 0)

    for dc in dc_list:

        figname = 'cd'+cd+'dc'+str(dc)+'_rmader_comm_delay_histogram.png'
        source_dir = "/home/kota/data/bags" # change the source dir accordingly #10 agents 
        if is_oldmader:
            source_bags = source_dir + "/oldmader/cd"+cd+"ms/*.bag" # change the source dir accordingly #10 agents
            is_oldmader = False
        else:
            source_bags = source_dir + "/rmader/cd"+cd+"msdc"+str(dc)+"ms/*.bag" # change the source dir accordingly #10 agents

        rosbag_list = glob.glob(source_bags)
        rosbag_list.sort() #alphabetically order
        rosbag = []
        comm_delay = []

        # comm_delay you use
        input_comm_delay = dc/1000

        for bag in rosbag_list:
            rosbag.append(bag)

        print(rosbag)

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

        if is_histogram:
            percentile = scipy.stats.percentileofscore(comm_delay_arr, input_comm_delay, kind='mean')
            os.system('echo "cd='+cd+', dc='+str(dc)+':   '+str(input_comm_delay) + ' is ' + str(percentile) + '-th percentile" >> '+source_dir+'/comm_delay_percentile.txt')
            # print(comm_delay)
            max_comm_delay = max(comm_delay)

            fig = plt.figure()
            ax = fig.add_subplot()
            n, bins, patches = plt.hist(x=comm_delay, color="blue")
            plt.axvline(x=dc/1000, color="red")
            ax.set_xticks(np.arange(0.05,0.250,0.025))
            ax.set_xticklabels(np.arange(50,250,25))
            # plt.rcParams["font.family"] = "Times New Roman"
            plt.grid(axis='y', color='black', alpha=0.2)
            plt.title('Comm delay histogram \n max comm_delay is '+str(round(max_comm_delay*1000))+' [ms]')
            plt.xlabel("comm delay [ms]")
            plt.ylabel("count")
            plt.savefig('/home/kota/ws/src/mader/mader/other/sim/data/'+figname)
            # plt.show()
        else:
            # in case you wanna calculate the value of q-th percentile
            print(str(q) + "-th percentile value is " + str(numpy.percentile(comm_delay_arr, q)))
            if q > 0:
                q = q -25
            else:
                sys.exit()