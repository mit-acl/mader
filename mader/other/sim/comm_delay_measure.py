#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python comm_delay_measure.py

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
    # source_file = "~/Research/data/bags/mader/multi_agent/2022-06-23-20-02-26.bag" # change the source dir accordingly
    source_file = "/home/kota/Research/data/bags/mader/multi_agent/sim_00_2022-06-26-13-57-03.bag" # change the source dir accordingly #10 agents 

    rosbag_list = glob.glob(source_file)
    rosbag = []
    comm_delay = []

    for bag in rosbag_list:
        rosbag.append(bag)

    for i in range(len(rosbag)):

        b = bagreader(rosbag[i], verbose=False);
        
        for i in range(1,11):
            if i < 10:
                log_data = b.message_by_topic("/SQ0" + str(i) + "s/mader/comm_delay")
            else:
                log_data = b.message_by_topic("/SQ" + str(i) + "s/mader/comm_delay")

            log = pd.read_csv(log_data)

            for j in range(len(log.comm_delay)):
                comm_delay.append(log.comm_delay[j])

        # print(comm_delay)
        max_comm_delay = max(comm_delay)

        n, bins, patches = plt.hist(x=comm_delay)
        plt.grid(axis='y', color='black', alpha=0.2)
        plt.title('Comm delay histogram \n max comm_delay is '+str(round(max_comm_delay,3))+' [s]')
        plt.xlabel("comm delay [s]")
        plt.ylabel("count")
        plt.savefig('comm_delay_histogram.png')
        plt.show()


    # fig=plt.figure()
    # plt.plot(Time, vx)
    # fig.suptitle("v.x")       
    # plt.show()

    # fig=plt.figure()
    # plt.plot(Time, vy)
    # fig.suptitle("v.y")       
    # plt.show()

    # fig=plt.figure()
    # plt.plot(Time, vz)
    # fig.suptitle("v.z")       
    # plt.show()

    

    # generate figure
    # fig, ax = bagpy.create_fig(1)
    # ax[0].scatter(x="Time", y="v.x", data=log)
    # plt.show()