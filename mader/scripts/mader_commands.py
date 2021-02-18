#!/usr/bin/env python

# /* ----------------------------------------------------------------------------
#  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Jesus Tordesillas, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import rospy
from mader_msgs.msg import Mode
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import Pose, PoseStamped
from snapstack_msgs.msg import QuadFlightMode
#from behavior_selector.srv import MissionModeChange
import math

def quat2yaw(q):
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return yaw

class Behavior_Selector:

    def __init__(self):
        self.mode=Mode();
        self.pose = Pose();
        self.mode.mode=self.mode.ON_GROUND
        self.pubGoal = rospy.Publisher('goal', Goal, queue_size=1)
        self.pubMode = rospy.Publisher("mader/mode",Mode,queue_size=1,latch=True) #TODO Namespace
        self.pubClickedPoint = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1,latch=True)
        

        self.alt_taken_off = 1; #Altitude when hovering after taking off
        self.alt_ground = 0; #Altitude of the ground
        self.initialized=False;

    #In rospy, the callbacks are all of them in separate threads
    def stateCB(self, data):
        self.pose.position.x = data.pos.x
        self.pose.position.y = data.pos.y
        self.pose.position.z = data.pos.z
        self.pose.orientation = data.quat

        if(self.initialized==False):
            self.pubFirstGoal()
            self.initialized=True
            #self.takeOff() #hack to take off directly

    #Called when buttom pressed in the interface
    def globalflightmodeCB(self,req):
        if(self.initialized==False):
            print "Not initialized yet"
            return

        if req.mode == req.GO and self.mode.mode==self.mode.ON_GROUND:
            print "Taking off"
            self.takeOff()
            print "Take off done"

        if req.mode == req.KILL:
            print "Killing"
            self.kill()

        if req.mode == req.LAND and self.mode.mode==self.mode.GO:
            print "Landing"
            self.land()
            print "Landing done"


    def sendMode(self):
        self.mode.header.stamp = rospy.get_rostime()
        self.pubMode.publish(self.mode)


    def takeOff(self):
        goal=Goal();
        goal.p.x = self.pose.position.x;
        goal.p.y = self.pose.position.y;
        goal.p.z = self.pose.position.z;
        goal.yaw = quat2yaw(self.pose.orientation)
        #Note that self.pose.position is being updated in the parallel callback

        ######## Commented for simulations
        # while(  abs(self.pose.position.z-self.alt_taken_off)>0.1  ): 
        #     goal.pos.z = min(goal.pos.z+0.0035, self.alt_taken_off);
        #     #rospy.sleep(0.004) 
        #     self.sendGoal(goal)
        ######## 
        rospy.sleep(0.1) 
        self.mode.mode=self.mode.GO
        self.sendMode();

    def land(self):
        goal=Goal();
        goal.pos.x = self.pose.position.x;
        goal.pos.y = self.pose.position.y;
        goal.pos.z = self.pose.position.z;
        goal.yaw = quat2yaw(self.pose.orientation)

        #Note that self.pose.position is being updated in the parallel callback
        while(abs(self.pose.position.z-self.alt_ground)>0.1):
            goal.pos.z = max(goal.pos.z-0.0035, self.alt_ground);
            self.sendGoal(goal)
        #Kill motors once we are on the ground
        self.kill()

    def kill(self):
        goal=Goal();
        goal.pos.x = self.pose.position.x;
        goal.pos.y = self.pose.position.y;
        goal.pos.z = self.pose.position.z;
        goal.cut_power=True
        self.sendGoal(goal)
        self.mode.mode=self.mode.ON_GROUND
        self.sendMode()

    def sendGoal(self, goal):
        # goal.yaw = quat2yaw(self.pose.orientation)
        goal.header.stamp = rospy.get_rostime()
        # print("[mader_cmds.py] Sending goal.yaw=",goal.yaw);
        self.pubGoal.publish(goal)

    def pubFirstGoal(self):
        msg=PoseStamped()
        msg.pose.position.x=self.pose.position.x
        msg.pose.position.y=self.pose.position.y
        msg.pose.position.z=1.0
        msg.pose.orientation = self.pose.orientation
        msg.header.frame_id="world"
        msg.header.stamp = rospy.get_rostime()
        self.pubClickedPoint.publish(msg)

                  
def startNode():
    c = Behavior_Selector()
    #s = rospy.Service("/change_mode",MissionModeChange,c.srvCB)
    rospy.Subscriber("state", State, c.stateCB)
    rospy.Subscriber("/globalflightmode", QuadFlightMode, c.globalflightmodeCB)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('behavior_selector')  
    startNode()
    print "Behavior selector started" 