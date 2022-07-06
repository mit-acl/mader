#!/usr/bin/env python

#!/usr/bin/env python  
#Author: Kota Kondo
#Date: July 6 2022

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

class GoalReachedCheck:

    def __init__(self):


        rospy.sleep(3)

        # goal radius
        self.goal_radius = 0.25 # set by mader.yaml 

        # number of agents
        self.num_of_agents = 10

        self.initialized = False

        # state and term_goal
        self.state_pos = np.empty([self.num_of_agents,3])
        self.term_goal_pos = np.empty([self.num_of_agents,3])

        # publisher init
        self.goal_reached = GoalReached()
        self.pubIsGoalReached = rospy.Publisher('goal_reached', GoalReached, queue_size=1, latch=True)

        # rospy.sleep(10) #term_goal is published after 10 seconds

    # collision detection
    def goalReachedCheck(self, timer):
        is_goal_reached = True
        if self.initialized:
            for i in range(self.num_of_agents):
                if (LA.norm(self.state_pos[i,:] - self.term_goal_pos[i,:]) > self.goal_radius):
                    print(i)
                    print(self.state_pos)
                    print(self.term_goal_pos[i,:])
                    # print(LA.norm(self.state_pos[i,:] - self.term_goal_pos[i,:]))
                    is_goal_reached = False
                    break

            if is_goal_reached:
            # print('here')
                self.goal_reached.is_goal_reached = True
                self.pubIsGoalReached.publish(self.goal_reached)

    def SQ01stateCB(self, data):
        self.state_pos[0,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ02stateCB(self, data):
        self.state_pos[1,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ03stateCB(self, data):
        self.state_pos[2,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ04stateCB(self, data):
        self.state_pos[3,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ05stateCB(self, data):
        self.state_pos[4,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ06stateCB(self, data):
        self.state_pos[5,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ07stateCB(self, data):
        self.state_pos[6,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ08stateCB(self, data):
        self.state_pos[7,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ09stateCB(self, data):
        self.state_pos[8,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    def SQ10stateCB(self, data):
        self.state_pos[9,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])

    def SQ01term_goalCB(self, data):
        self.term_goal_pos[0,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.initialized = True
    def SQ02term_goalCB(self, data):
        self.term_goal_pos[1,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ03term_goalCB(self, data):
        self.term_goal_pos[2,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ04term_goalCB(self, data):
        self.term_goal_pos[3,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ05term_goalCB(self, data):
        self.term_goal_pos[4,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ06term_goalCB(self, data):
        self.term_goal_pos[5,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ07term_goalCB(self, data):
        self.term_goal_pos[6,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ08term_goalCB(self, data):
        self.term_goal_pos[7,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ09term_goalCB(self, data):
        self.term_goal_pos[8,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ10term_goalCB(self, data):
        self.term_goal_pos[9,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

def startNode():
    c = GoalReachedCheck()
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
    rospy.Subscriber("SQ01s/term_goal", PoseStamped, c.SQ01term_goalCB)
    rospy.Subscriber("SQ02s/term_goal", PoseStamped, c.SQ02term_goalCB)
    rospy.Subscriber("SQ03s/term_goal", PoseStamped, c.SQ03term_goalCB)
    rospy.Subscriber("SQ04s/term_goal", PoseStamped, c.SQ04term_goalCB)
    rospy.Subscriber("SQ05s/term_goal", PoseStamped, c.SQ05term_goalCB)
    rospy.Subscriber("SQ06s/term_goal", PoseStamped, c.SQ06term_goalCB)
    rospy.Subscriber("SQ07s/term_goal", PoseStamped, c.SQ07term_goalCB)
    rospy.Subscriber("SQ08s/term_goal", PoseStamped, c.SQ08term_goalCB)
    rospy.Subscriber("SQ09s/term_goal", PoseStamped, c.SQ09term_goalCB)
    rospy.Subscriber("SQ10s/term_goal", PoseStamped, c.SQ10term_goalCB)
    rospy.Timer(rospy.Duration(1), c.goalReachedCheck)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('goalReachedCheck')
    startNode()