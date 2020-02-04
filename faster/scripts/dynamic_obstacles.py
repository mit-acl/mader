#!/usr/bin/env python

#Jesus Tordesillas Torres, December 2019

#This files plots in gazebo with the position and orientation of the drone according to the desired position and acceleration specified in the goal topic

import roslib
import rospy
import math
from faster_msgs.msg import DynTraj
from snapstack_msgs.msg import QuadGoal, State
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point


import numpy as np
from numpy import linalg as LA
import random

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

        self.pubTraj = rospy.Publisher('/trajs', DynTraj, queue_size=1, latch=True)

        self.pubShapes = rospy.Publisher('/shapes', Marker, queue_size=1, latch=True)

        self.timer = rospy.Timer(rospy.Duration(0.001), self.pubTF)

        self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.num_of_objects = 21;

        self.x_all=[];
        self.y_all=[];
        self.z_all=[];
        for i in range(self.num_of_objects):
            self.x_all.append(10*random.random());
            self.y_all.append(10*random.random());
            self.z_all.append(2);
        # self.state.quat.x = 0
        # self.state.quat.y = 0
        # self.state.quat.z = 0
        # self.state.quat.w = 1

        # self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        rospy.sleep(0.5)

        # self.state.header.frame_id="world"
        # self.pubState.publish(self.state)  

    def pubTF(self, timer):
        br = tf.TransformBroadcaster()

        marker=Marker()
        marker.header.frame_id="world"
        marker.type=marker.CUBE_LIST;
        marker.action=marker.ADD;
        marker.color.r=0
        marker.color.g=0
        marker.color.b=1
        marker.color.a=1

        for i in range(self.num_of_objects):
            t_ros=rospy.Time.now()
            t=rospy.get_time(); #Same as before, but it's float
            [x_string, y_string, z_string] = self.trefoil(self.x_all[i], self.y_all[i], self.z_all[i], 1.0, 1.0, 1.0, self.x_all[i]) #offset=x
            x = eval(x_string)
            y = eval(y_string)
            z = eval(z_string)
            dynamic_trajectory_msg=DynTraj(); 

            dynamic_trajectory_msg.header.stamp= t_ros;
            dynamic_trajectory_msg.function = [x_string, y_string, z_string]
            dynamic_trajectory_msg.bbox = [0.1, 0.1, 2]
            dynamic_trajectory_msg.pos.x=x #Current position
            dynamic_trajectory_msg.pos.y=y #Current position
            dynamic_trajectory_msg.pos.z=z #Current position

            dynamic_trajectory_msg.id = i #Current id

            self.pubTraj.publish(dynamic_trajectory_msg)
            br.sendTransform((x, y, z), (0,0,0,1), t_ros, self.name+str(i), "world")


            #If you want to move the objets in gazebo
            # gazebo_state = ModelState()
            # gazebo_state.model_name = str(i)
            # gazebo_state.pose.position.x = x
            # gazebo_state.pose.position.y = y
            # gazebo_state.pose.position.z = z
            # gazebo_state.reference_frame = "world" 
            # self.pubGazeboState.publish(gazebo_state)  

            #If you want to see the objets in rviz
            marker.scale.x=dynamic_trajectory_msg.bbox[0]
            marker.scale.y=dynamic_trajectory_msg.bbox[1]
            marker.scale.z=dynamic_trajectory_msg.bbox[2]

            point=Point()
            point.x=x;
            point.y=y;
            point.z=z;
            marker.points.append(point);

        self.pubShapes.publish(marker)



    # Trefoil knot, https://en.wikipedia.org/wiki/Trefoil_knot
    def trefoil(self,x,y,z,scale_x, scale_y, scale_z, offset):

        x_string='(sin(t +'+str(offset)+') + 2 * sin(2 * t +'+str(offset)+'))/' + str(scale_x) +'+' + str(x); #'2*sin(t)' 
        y_string='(cos(t +'+str(offset)+') - 2 * cos(2 * t +'+str(offset)+'))/' + str(scale_y) +'+' + str(y); #'2*cos(t)' 
        z_string='0.5'#'-sin(3 * t +'+str(offset)+')/' +str(scale_z) + '+' + str(z);                                #'1'        

        return [x_string, y_string, z_string]


             

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
