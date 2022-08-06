#!/usr/bin/env python

#!/usr/bin/env python  
#Author: Kota Kondo
#Date: July 15, 2022

#you can get transformation btwn two agents and if they are too close (violating bbox) then it prints out warning
#the reason why we use tf instead of snapstack_msgs/State is two agents publish their states asynchrnonously and therefore comparing
#these states (with slightly different timestamp) is not accurate position comparison. Whereas tf always compares two states with the same time stamp

import math
import os
import sys
import time
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped
from snapstack_msgs.msg import State
from mader_msgs.msg import GoalReached
import numpy as np
from random import *
import tf2_ros
from numpy import linalg as LA
import pandas as pd 

class AveDistance:
    
    def __init__(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.sleep(3) #Important, if not it won't work

        # bbox size
        self.num_of_agents = rospy.get_param('~num_of_agents', 10)

        # matrix that saves sum of distance
        self.dist_matrix = np.zeros([self.num_of_agents, self.num_of_agents])

        # count how many dist we get
        self.cnt = np.zeros([self.num_of_agents, self.num_of_agents])

        # folder location
        self.folder_loc = rospy.get_param('~folder_loc')

        # sim number
        self.sim = rospy.get_param('~sim')
        if self.sim <=9:
            self.sim = "0" + str(self.sim)

        # subscriber
        self.goal_reached = GoalReached()
        self.subGoalReached = rospy.Subscriber('goal_reached', GoalReached, self.GoalReachedCB)

        self.initialized = False
        self.initialized_mat = [False for i in range(self.num_of_agents)]

        self.state_pos = np.empty([self.num_of_agents,3])

    def GoalReachedCB(self, data):
        for i in range(self.num_of_agents):
            for j in range(i+1,self.num_of_agents):
                self.dist_matrix[i,j] = self.dist_matrix[i,j] / self.cnt[i,j]
        # print(self.dist_matrix)
        pd.DataFrame(self.dist_matrix).to_csv(str(self.folder_loc)+'/sim_'+str(self.sim)+'.csv')
        os.system("rosnode kill ave_distance");

    # collision detection
    def AveDistanceCalculate(self, timer):
        
        if not self.initialized:
            initialized = True
            for k in range(self.num_of_agents):
                if not self.initialized_mat[k]:
                    initialized = False
                    break
            self.initialized = initialized


        if self.initialized:
            for i in range(self.num_of_agents):
                for j in range(i+1,self.num_of_agents):

                    if i<9:
                        agent1 = "SQ0" + str(i+1) + "s" 
                    else:
                        agent1 = "SQ" + str(i+1) + "s" 

                    if j<9:
                        agent2 = "SQ0" + str(j+1) + "s" 
                    else:
                        agent2 = "SQ" + str(j+1) + "s" 
                    
                    # trans = self.get_transformation(agent1, agent2)

                    x_diff = abs(self.state_pos[i,0] - self.state_pos[j,0])
                    y_diff = abs(self.state_pos[i,1] - self.state_pos[j,1])
                    z_diff = abs(self.state_pos[i,2] - self.state_pos[j,2])

                    self.dist_matrix[i,j] = self.dist_matrix[i,j] + LA.norm(np.array([x_diff, y_diff, z_diff]))
                    self.cnt[i,j] = self.cnt[i,j] + 1                    

                    # if trans is not None:

                    #     # print(LA.norm(np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])))

                    #     self.dist_matrix[i,j] = self.dist_matrix[i,j] + LA.norm(np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]))
                    #     self.cnt[i,j] = self.cnt[i,j] + 1

    def SQ01stateCB(self, data):
        self.state_pos[0,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        # print(LA.norm(self.state_pos))
        if self.initialized_mat[0] == False and LA.norm(self.state_pos[0,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[0] = True
    def SQ02stateCB(self, data):
        self.state_pos[1,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        if self.initialized_mat[1] == False and LA.norm(self.state_pos[1,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[1] = True
    def SQ03stateCB(self, data):
        self.state_pos[2,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        if self.initialized_mat[2] == False and LA.norm(self.state_pos[2,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[2] = True
    def SQ04stateCB(self, data):
        self.state_pos[3,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        if self.initialized_mat[3] == False and LA.norm(self.state_pos[3,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[3] = True
    def SQ05stateCB(self, data):
        self.state_pos[4,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        if self.initialized_mat[4] == False and LA.norm(self.state_pos[4,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[4] = True
    def SQ06stateCB(self, data):
        self.state_pos[5,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        if self.initialized_mat[5] == False and LA.norm(self.state_pos[5,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[5] = True
    def SQ07stateCB(self, data):
        self.state_pos[6,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        if self.initialized_mat[6] == False and LA.norm(self.state_pos[6,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[6] = True
    def SQ08stateCB(self, data):
        self.state_pos[7,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        if self.initialized_mat[7] == False and LA.norm(self.state_pos[7,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[7] = True
    def SQ09stateCB(self, data):
        self.state_pos[8,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        if self.initialized_mat[8] == False and LA.norm(self.state_pos[8,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[8] = True
    def SQ10stateCB(self, data):
        self.state_pos[9,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
        if self.initialized_mat[9] == False and LA.norm(self.state_pos[9,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[9] = True

    def get_transformation(self, source_frame, target_frame):

        # get the tf at first available time
        try:
            transformation = self.tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(0.1))
            return transformation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            # rospy.logerr("Unable to find the transformation")

def startNode():
    c = AveDistance()
    rospy.Subscriber("SQ01s/state", State, c.SQ01stateCB)
    rospy.Subscriber("SQ02s/state", State, c.SQ02stateCB)
    rospy.Subscriber("SQ03s/state", State, c.SQ03stateCB)
    rospy.Subscriber("SQ04s/state", State, c.SQ04stateCB)
    rospy.Subscriber("SQ05s/state", State, c.SQ05stateCB)
    rospy.Subscriber("SQ06s/state", State, c.SQ06stateCB)
    rospy.Subscriber("SQ07s/state", State, c.SQ07stateCB)
    rospy.Subscriber("SQ08s/state", State, c.SQ08stateCB)
    rospy.Subscriber("SQ09s/state", State, c.SQ09stateCB)
    rospy.Subscriber("SQ10s/state", State, c.SQ10stateCB)
    rospy.Subscriber("SQ01s/state", State, c.SQ01stateCB)

    rospy.Timer(rospy.Duration(0.01), c.AveDistanceCalculate)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('AveDistance')
    startNode()