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
    source_dir = "/home/kota/Research/bags/rmader/*.bag" # change the source dir accordingly #10 agents 

    rosbag_list = glob.glob(source_dir)
    rosbag = []

    for bag in rosbag_list:
        rosbag.append(bag)

    for i in range(len(rosbag)):

        b = bagreader(rosbag[i], verbose=False)
        print(rosbag[i])
        try:
        	log_data = b.message_by_topic("is_collided")
        	print('collisiondetected')
        except:
        	print("no collision")

