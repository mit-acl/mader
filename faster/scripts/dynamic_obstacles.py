#!/usr/bin/env python

#Jesus Tordesillas Torres, December 2019

#This files plots in gazebo with the position and orientation of the drone according to the desired position and acceleration specified in the goal topic

import roslib
import rospy
import math
from faster_msgs.msg import DynTraj
from snapstack_msgs.msg import QuadGoal, State
from gazebo_msgs.msg import ModelState
import numpy as np
from numpy import linalg as LA

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from pyquaternion import Quaternion
import tf

from math import sin, cos, tan



class FakeSim:

    def __init__(self):
        self.state=State()

        self.timer = rospy.Timer(rospy.Duration(0.01), self.pubTF)
        name = rospy.get_namespace()
        self.name = name[1:-1]

        self.pubTraj = rospy.Publisher('traj', DynTraj, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.pubTF)

        # self.state.quat.x = 0
        # self.state.quat.y = 0
        # self.state.quat.z = 0
        # self.state.quat.w = 1

        # self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        rospy.sleep(1.0)

        # self.state.header.frame_id="world"
        # self.pubState.publish(self.state)  

    def pubTF(self, timer):
        br = tf.TransformBroadcaster()
        # Trefoil knot, https://en.wikipedia.org/wiki/Trefoil_knot
        t_ros=rospy.Time.now()
        t=rospy.get_time(); #Same as before, but it's float
        x_string='(sin(t) + 2 * sin(2 * t))/1.0 -5';
        y_string='(cos(t) - 2 * cos(2 * t))/1.0';
        z_string='-sin(3 * t) +1';
        x = eval(x_string)
        y = eval(y_string)
        z = eval(z_string)
        dynamic_trajectory_msg=DynTraj();
        dynamic_trajectory_msg.header.stamp= t_ros;
        dynamic_trajectory_msg.function = [x_string, y_string, z_string]
        dynamic_trajectory_msg.bbox = [0.1, 0.1, 0.1]
        self.pubTraj.publish(dynamic_trajectory_msg)
        br.sendTransform((x, y, z), (0,0,0,1), t_ros, self.name, "vicon")


             

def startNode():
    c = FakeSim()

    rospy.spin()

if __name__ == '__main__':

    ns = rospy.get_namespace()
    try:
        rospy.init_node('relay')
        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is tyipcally accomplished in a launch file.")
            rospy.logfatal("Command line: ROS_NAMESPACE=mQ01 $ rosrun quad_control joy.py")
        else:
            print "Starting joystick teleop node for: " + ns
            startNode()
    except rospy.ROSInterruptException:
        pass
