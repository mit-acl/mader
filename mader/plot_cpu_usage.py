#!/usr/bin/env python  
#Author:Kota Kondo

#Instructions:
#get cpu usage data using record_cpu.sh, which will create a log file containg total cpu usage

import bagpy
from bagpy import bagreader
import pandas as pd
import sys
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

def main(argv):

    data = []
    time = 0
    with open(argv) as source_file:
        for line in source_file:
            data.append(float(line))
            time = time + 1

    Time = range(time)
    fig=plt.figure()
    plt.plot(Time, data)
    plt.xlabel("time [s]")
    plt.ylabel("CPU usage %")
    plt.grid(color='black', alpha=0.2, linewidth=0.5)
    fig.suptitle("CPU usage")    
    plt.show()

if __name__ == '__main__':
    main(sys.argv[1])

    