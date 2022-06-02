#!/usr/bin/env python

import math
import os
import time
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped
from snapstack_msgs.msg import State
import numpy as np
from random import * 

class CollisionDetector:

    def __init__(self):

        # bbox size
        self.bbox_x = rospy.get_param('bbox_x', 1.0) #default value is 1.0 
        self.bbox_y = rospy.get_param('bbox_y', 1.0) #default value is 1.0 
        self.bbox_z = rospy.get_param('bbox_z', 1.5) #default value is 1.5

        self.initialized = False

        self.state_pos = np.empty([6,3])

    # collision detection
    def collisionDetect(self, timer):
        
        if self.initialized:
            for i in range(5):
                for j in range(i+1,6):
                    if (abs(self.state_pos[i,0] - self.state_pos[j,0]) < self.bbox_x 
                        and abs(self.state_pos[i,1] - self.state_pos[j,1]) < self.bbox_y 
                        and abs(self.state_pos[i,2] - self.state_pos[j,2]) < self.bbox_z):
                        print("difference in x is " + str(abs(self.state_pos[i,0] - self.state_pos[j,0])))
                        print("difference in y is " + str(abs(self.state_pos[i,1] - self.state_pos[j,1])))
                        print("difference in z is " + str(abs(self.state_pos[i,2] - self.state_pos[j,2])))
                        print("agent" + str(i+1) + " and " + str(j+1) + " collide")

    def SQ01stateCB(self, data):
        self.state_pos[0,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.initialized = True
    def SQ02stateCB(self, data):
        self.state_pos[1,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ03stateCB(self, data):
        self.state_pos[2,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ04stateCB(self, data):
        self.state_pos[3,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ05stateCB(self, data):
        self.state_pos[4,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ06stateCB(self, data):
        self.state_pos[5,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

def startNode():
    c = CollisionDetector()
    rospy.Subscriber("SQ01s/state", PoseStamped, c.SQ01stateCB)
    rospy.Subscriber("SQ02s/state", PoseStamped, c.SQ02stateCB)
    rospy.Subscriber("SQ03s/state", PoseStamped, c.SQ03stateCB)
    rospy.Subscriber("SQ04s/state", PoseStamped, c.SQ04stateCB)
    rospy.Subscriber("SQ05s/state", PoseStamped, c.SQ05stateCB)
    rospy.Subscriber("SQ06s/state", PoseStamped, c.SQ06stateCB)
    rospy.Timer(rospy.Duration(0.01), c.collisionDetect)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('CollisionDetector')
    startNode()
