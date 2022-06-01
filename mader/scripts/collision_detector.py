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

        self.world_pos = np.empty([6,3])

    # collision detection
    def collisionDetect(self, timer):
        
        if self.initialized:
            for i in range(5):
                if (abs(self.world_pos[i,0] - self.world_pos[i+1,0]) < self.bbox_x or abs(self.world_pos[i,1] - self.world_pos[i+1,1]) < self.bbox_y or abs(self.world_pos[i,2] - self.world_pos[i+1,2]) < self.bbox_z):
                    print("agent" + str(i+1) + "and agent" + str(i+1) + " collide")

    def SQ01worldCB(self, data):
        self.world_pos[0,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.initialized = True
    def SQ02worldCB(self, data):
        self.world_pos[1,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ03worldCB(self, data):
        self.world_pos[2,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ04worldCB(self, data):
        self.world_pos[3,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ05worldCB(self, data):
        self.world_pos[4,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    def SQ06worldCB(self, data):
        self.world_pos[5,0:3] = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

def startNode():
    c = CollisionDetector()
    rospy.Subscriber("SQ01s/world", PoseStamped, c.SQ01worldCB)
    rospy.Subscriber("SQ02s/world", PoseStamped, c.SQ02worldCB)
    rospy.Subscriber("SQ03s/world", PoseStamped, c.SQ03worldCB)
    rospy.Subscriber("SQ04s/world", PoseStamped, c.SQ04worldCB)
    rospy.Subscriber("SQ05s/world", PoseStamped, c.SQ05worldCB)
    rospy.Subscriber("SQ06s/world", PoseStamped, c.SQ06worldCB)
    rospy.Timer(rospy.Duration(0.01), c.collisionDetect)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('CollisionDetector')
    startNode()
