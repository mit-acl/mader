#!/usr/bin/python

# Kota Kondo, kkondo@mit.edu, Aug 19, 2022

# Instructions:
# python statistics.py "...../*.bags"

# Need to install tf_bag and termcolor
# REF:https://github.com/IFL-CAMP/tf_bag
#   git clone https://github.com/IFL-CAMP/tf_bag.git
#   catkin build
# pip install termcolor

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import rospy
import numpy as np
import rosbag
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

import os
import sys

from tf_bag import BagTfTransformer
import numpy as np
from colorama import init, Fore, Back, Style

from termcolor import colored

import re
import glob

#rospy.init_node('talker', anonymous=True)

# home_dir = "/media/kota/T7/gurobi_nlopt/"
home_dir = "/home/kota/data/gurobi_nlopt/"

num_of_agents=10

source_dir = "/home/kota/data/gurobi_nlopt/bags/*.bag"
# get the bags
list_of_bags = glob.glob(source_dir)

ave_completion_time = 0.0

for name_bag in list_of_bags:
    bag = rosbag.Bag(name_bag)
    computation_time_cnt = 0
    for topic, msg, t in bag.read_messages(topics='/SQ01s/mader/computation_time'):
        ave_completion_time = ave_completion_time + msg.computation_time
        computation_time_cnt = computation_time_cnt + 1

    ave_completion_time /= computation_time_cnt

    rospy.sleep(4.0)
    bag.close()

# output txt file
ave_completion_time /= len(list_of_bags)

os.system('echo "----------------------------------------------------------------------------------" >> '+home_dir+'computation_time.txt')
os.system('echo "'+str(source_dir)+'" >> '+home_dir+'computation_time.txt')
os.system('echo "----------------------------------------------------------------------------------" >> '+home_dir+'computation_time.txt')
os.system('echo " computation time [ms]'+str(round(ave_completion_time,2))+'" >> '+home_dir+'computation_time.txt')
