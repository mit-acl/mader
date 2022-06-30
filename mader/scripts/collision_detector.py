#!/usr/bin/env python

#!/usr/bin/env python  
#Author: Kota Kondo
#Date: June 3. 2022

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
from mader_msgs.msg import Collision
import numpy as np
from random import *
import tf2_ros

class CollisionDetector:

    def __init__(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.sleep(3) #Important, if not it won't work

        # tolerance
        self.tol = 0.001

        # bbox size
        self.bbox_x = rospy.get_param('~bbox_x', 0.25) - self.tol #default value is 1.0 
        self.bbox_y = rospy.get_param('~bbox_y', 0.25) - self.tol #default value is 1.0 
        self.bbox_z = rospy.get_param('~bbox_z', 0.25) - self.tol #default value is 1.5
        self.num_of_agents = rospy.get_param('~num_of_agents')

        self.initialized = True

        self.state_pos = np.empty([6,3])

        # publisher init
        self.collision=Collision()
        self.pubIsCollided = rospy.Publisher('is_collided', Collision, queue_size=1, latch=True)

    # collision detection
    def collisionDetect(self, timer):
        
        if self.initialized:
            for i in range(1,self.num_of_agents):
                for j in range(i+1,self.num_of_agents+1):

                    if i<=9:
                        agent1 = "SQ0" + str(i) + "s" 
                    else:
                        agent1 = "SQ" + str(i) + "s" 

                    if j<=9:
                        agent2 = "SQ0" + str(j) + "s" 
                    else:
                        agent2 = "SQ" + str(j) + "s" 
                    
                    trans = self.get_transformation(agent1, agent2)

                    if trans is not None:

                        # print(str(agent1) + " and " + str(agent2) + ": " + str(trans.transform.translation.x))
                    
                        if (abs(trans.transform.translation.x) < self.bbox_x
                            and abs(trans.transform.translation.y) < self.bbox_y
                            and abs(trans.transform.translation.z) < self.bbox_z):
                            
                            self.collision.is_collided = True
                            self.collision.agent1 = trans.header.frame_id
                            self.collision.agent2 = trans.child_frame_id
                            self.pubIsCollided.publish(self.collision)

                            print("collision btwn " + trans.header.frame_id + " and " + trans.child_frame_id)

                            max_dist = max(abs(trans.transform.translation.x), abs(trans.transform.translation.y), abs(trans.transform.translation.z))

                            print("violation dist is " + str(max_dist))

                        # if (abs(self.state_pos[i,0] - self.state_pos[j,0]) < self.bbox_x 
                        #     and abs(self.state_pos[i,1] - self.state_pos[j,1]) < self.bbox_y 
                        #     and abs(self.state_pos[i,2] - self.state_pos[j,2]) < self.bbox_z):
                        #     print("difference in x is " + str(abs(self.state_pos[i,0] - self.state_pos[j,0])))
                        #     print("difference in y is " + str(abs(self.state_pos[i,1] - self.state_pos[j,1])))
                        #     print("difference in z is " + str(abs(self.state_pos[i,2] - self.state_pos[j,2])))
                        #     print("agent" + str(i+1) + " and " + str(j+1) + " collide")

    # def SQ01stateCB(self, data):
    #     self.state_pos[0,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    #     self.initialized = True
    # def SQ02stateCB(self, data):
    #     self.state_pos[1,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    # def SQ03stateCB(self, data):
    #     self.state_pos[2,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    # def SQ04stateCB(self, data):
    #     self.state_pos[3,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    # def SQ05stateCB(self, data):
    #     self.state_pos[4,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])
    # def SQ06stateCB(self, data):
    #     self.state_pos[5,0:3] = np.array([data.pos.x, data.pos.y, data.pos.z])

    def get_transformation(self, source_frame, target_frame):

        # get the tf at first available time
        try:
            transformation = self.tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(0.1))
            return transformation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            # rospy.logerr("Unable to find the transformation")

def startNode():
    c = CollisionDetector()
    # rospy.Subscriber("SQ01s/state", State, c.SQ01stateCB)
    # rospy.Subscriber("SQ02s/state", State, c.SQ02stateCB)
    # rospy.Subscriber("SQ03s/state", State, c.SQ03stateCB)
    # rospy.Subscriber("SQ04s/state", State, c.SQ04stateCB)
    # rospy.Subscriber("SQ05s/state", State, c.SQ05stateCB)
    # rospy.Subscriber("SQ06s/state", State, c.SQ06stateCB)
    rospy.Timer(rospy.Duration(0.01), c.collisionDetect)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('CollisionDetector')
    startNode()