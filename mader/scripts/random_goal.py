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

        # initialization done?
        self.is_init_pos = False

        # term_goal init
        self.term_goal=PoseStamped()
        self.term_goal.header.frame_id='world'
        self.pubTermGoal = rospy.Publisher('term_goal', PoseStamped, queue_size=1, latch=True)
        
        # state_pos init ()
        self.state_pos=np.array([0.0, 0.0, 0.0])

        # every 0.01 sec timerCB is called back
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timerCB)

        # send goal
        self.sendGoal()

        # set initial time and how long the demo is
        self.time_init = rospy.get_rostime()
        self.total_secs = 60.0; # sec

        # home yet?
        self.is_home = False

    def timerCB(self, tmp):
        
        # term_goal in array form
        self.term_goal_pos=np.array([self.term_goal.pose.position.x,self.term_goal.pose.position.y,self.term_goal.pose.position.z])

        # distance
        dist=np.linalg.norm(self.term_goal_pos-self.state_pos)
        #print("dist=", dist)

        # check distance and if it's close enough publish new term_goal
        dist_limit = 0.3
        if (dist < dist_limit):
            if not self.is_home:
                self.sendGoal()

        # check if we should go home
        duration = rospy.get_rostime() - self.time_init
        if (duration.to_sec() > self.total_secs):
            self.is_home = True
            self.sendGoalHome()

    def sendGoal(self):

        # highbay boundary
        x_min = -4.1
        x_max = 4.1
        y_min = -4.1
        y_max = 4.1
        z_min = 0.9
        z_max = 3.0

        # set random goals
        self.term_goal.pose.position.x = x_min + (x_max - x_min) * random()
        self.term_goal.pose.position.y = y_min + (y_max - y_min) * random()
        self.term_goal.pose.position.z = z_min + (z_max - z_min) * random()

        self.pubTermGoal.publish(self.term_goal)      

        return

    def sendGoalHome(self):

        # set home goals
        self.term_goal.pose.position.x = self.init_pos[0]
        self.term_goal.pose.position.y = self.init_pos[1]
        self.term_goal.pose.position.z = 1.8

        self.pubTermGoal.publish(self.term_goal)  

        print ("Home Return")
        print("term_goal x = ", self.term_goal.pose.position.x)      
        print("term_goal y = ", self.term_goal.pose.position.y)      
        print("term_goal z = ", self.term_goal.pose.position.z)      

        return

    def stateCB(self, data):
        if not self.is_init_pos:
            self.init_pos = np.array([data.pos.x, data.pos.y, data.pos.z])
            self.is_init_pos = True

        self.state_pos = np.array([data.pos.x, data.pos.y, data.pos.z])

def startNode():
    c = TermGoalSender()
    rospy.Subscriber("state", State, c.stateCB)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('TermGoalSender')
    startNode()

