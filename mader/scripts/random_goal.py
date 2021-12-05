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

class TermGoalSender:

    def __init__(self):

        # term_goal init
        self.term_goal=PoseStamped()
        self.term_goal.header.frame_id='world'
        self.pubTermGoal = rospy.Publisher('term_goal', PoseStamped, queue_size=1, latch=True)
        
        # state_pos init ()
        self.state_pos=np.array([0.0, 0.0, 0.0])

        # every 0.01 sec timerCB is called back
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timerCB)
        self.sign=1.0;

        # send goal
        self.sendGoal()

    def timerCB(self, tmp):
        
        # term_goal in array form
        self.term_goal_pos=np.array([self.term_goal.pose.position.x,self.term_goal.pose.position.y,self.term_goal.pose.position.z])

        # distance
        dist=np.linalg.norm(self.term_goal_pos-self.state_pos)
        print("dist=", dist)

        # check distance and if it's close enough publish new term_goal
        dist_limit = 0.2
        if(dist < dist_limit):
            self.sendGoal()

    def sendGoal(self):

        # highbay boundary
        x_min = -4.25
        x_max = 4.5
        y_min = -3.5
        y_max = 4.25
        z_min = 1.0
        z_max = 4.0

        # set random goals
        self.term_goal.pose.position.x = x_min + (x_max - x_min) * random()
        self.term_goal.pose.position.y = y_min + (y_max - y_min) * random()
        self.term_goal.pose.position.z = z_min + (z_max - z_min) * random()

        self.pubTermGoal.publish(self.term_goal)  

        print ("Goal sent!!")      

        return

    def stateCB(self, data):
        self.state_pos=np.array([data.pos.x, data.pos.y, data.pos.z])

def startNode():
    c = TermGoalSender()
    rospy.Subscriber("state", State, c.stateCB)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('TermGoalSender')
    startNode()

