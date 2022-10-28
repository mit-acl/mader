#!/usr/bin/env python

# /* ----------------------------------------------------------------------------
#  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Jesus Tordesillas, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

#This files plots in gazebo with the position and orientation of the drone according to the desired position and acceleration specified in the goal topic

import random
import roslib
import rospy
import math
from mader_msgs.msg import DynTraj
from snapstack_msgs.msg import Goal, State
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

from std_msgs.msg import ColorRGBA

import numpy as np
from numpy import linalg as LA
import random

from tf.transformations import quaternion_from_euler, euler_from_quaternion

# from pyquaternion import Quaternion
import tf

from math import sin, cos, tan


import copy 
import sys

color_static=ColorRGBA(r=0,g=0,b=1,a=1);
color_dynamic=ColorRGBA(r=1,g=0,b=0,a=1);

class HwObstacle:

    def __init__(self):

        self.state=State()
        name = rospy.get_namespace()
        self.name = name[1:-1]

        available_meshes_static=["package://mader/meshes/ConcreteDamage01b/model3.dae", "package://mader/meshes/ConcreteDamage01b/model2.dae"]
        available_meshes_dynamic=["package://mader/meshes/ConcreteDamage01b/model4.dae"]
        # available_meshes=["package://mader/meshes/ConcreteDamage01b/model3.dae"]
        
        self.r = 3.4
        self.type = "dynamic"
        self.meshes = random.choice(available_meshes_dynamic)
        self.bbox = [1.0, 1.0, 1.5]

        self.pubTraj = rospy.Publisher('/trajs', DynTraj, queue_size=1, latch=True)
        self.pubShapes_static = rospy.Publisher('/shapes_static', Marker, queue_size=1, latch=True)
        self.pubShapes_static_mesh = rospy.Publisher('/shapes_static_mesh', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic_mesh = rospy.Publisher('/shapes_dynamic_mesh', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic = rospy.Publisher('/shapes_dynamic', Marker, queue_size=1, latch=True)
        self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        rospy.sleep(0.5)

    # Circle
    def circle(self, r):
        x_string=str(r)+'*cos(t)' 
        y_string=str(r)+'*sin(t)' 
        z_string='1.0'        
        return [x_string, y_string, z_string]

    def pubTF(self, timer):
        br = tf.TransformBroadcaster()

        marker_tmp=Marker();
        marker_tmp.header.frame_id="world"
        marker_tmp.type=marker_tmp.CUBE_LIST;
        marker_tmp.action=marker_tmp.ADD;

        marker_static=copy.deepcopy(marker_tmp);
        marker_dynamic=copy.deepcopy(marker_tmp);

        marker_dynamic.color=color_dynamic;
        # marker_dynamic.scale.x=self.world.bbox_dynamic[0]
        # marker_dynamic.scale.y=self.world.bbox_dynamic[1]
        # marker_dynamic.scale.z=self.world.bbox_dynamic[2]

        marker_static.color=color_static;

        marker_array_static_mesh=MarkerArray();
        marker_array_dynamic_mesh=MarkerArray();

        t_ros=rospy.Time.now()
        t=rospy.get_time(); #Same as before, but it's float

        dynamic_trajectory_msg=DynTraj(); 
        
        [x_string, y_string, z_string] = self.circle(self.r) 
        dynamic_trajectory_msg.bbox = self.bbox;
        marker_dynamic.scale.x=self.bbox[0]
        marker_dynamic.scale.y=self.bbox[1]
        marker_dynamic.scale.z=self.bbox[2]

        x = eval(x_string)
        y = eval(y_string)
        z = eval(z_string)

        dynamic_trajectory_msg.is_agent=False;
        dynamic_trajectory_msg.header.stamp= t_ros;
        dynamic_trajectory_msg.function = [x_string, y_string, z_string]
        dynamic_trajectory_msg.pos.x=x #Current position
        dynamic_trajectory_msg.pos.y=y #Current position
        dynamic_trajectory_msg.pos.z=z #Current position

        dynamic_trajectory_msg.id = 4000 #Current id 4000 to avoid interference with ids from agents #TODO

        self.pubTraj.publish(dynamic_trajectory_msg)
        br.sendTransform((x, y, z), (0,0,0,1), t_ros, self.name+str(dynamic_trajectory_msg.id), "world")


        #If you want to move the objets in gazebo
        # gazebo_state = ModelState()
        # gazebo_state.model_name = str(i)
        # gazebo_state.pose.position.x = x
        # gazebo_state.pose.position.y = y
        # gazebo_state.pose.position.z = z
        # gazebo_state.reference_frame = "world" 
        # self.pubGazeboState.publish(gazebo_state)  

        #If you want to see the objects in rviz
        point=Point()
        point.x=x;
        point.y=y;
        point.z=z;

        ##########################
        marker=Marker();
        marker.id=1;
        marker.ns="mesh";
        marker.header.frame_id="world"
        marker.type=marker.MESH_RESOURCE;
        marker.action=marker.ADD;

        marker.pose.position.x=x
        marker.pose.position.y=y
        marker.pose.position.z=z
        marker.pose.orientation.x=0.0;
        marker.pose.orientation.y=0.0;
        marker.pose.orientation.z=0.0;
        marker.pose.orientation.w=1.0;
        marker.lifetime = rospy.Duration.from_sec(0.0);
        marker.mesh_use_embedded_materials=True
        # marker.mesh_resource=self.meshes[0]

        marker_dynamic.points.append(point);

        marker.scale.x=self.bbox[0];
        marker.scale.y=self.bbox[1];
        marker.scale.z=self.bbox[2];

        marker_array_dynamic_mesh.markers.append(marker);

        self.pubShapes_dynamic_mesh.publish(marker_array_dynamic_mesh)
        self.pubShapes_dynamic.publish(marker_dynamic)

        # if(self.already_published_static_shapes==False):

        self.pubShapes_static_mesh.publish(marker_array_static_mesh)
        self.pubShapes_static.publish(marker_static)

        # self.already_published_static_shapes=True;


def startNode():
    c = HwObstacle()
    rospy.Timer(rospy.Duration(0.01), c.pubTF)
    rospy.spin()

if __name__ == '__main__':

    # TODO: Use instead https://docs.python.org/3.3/library/argparse.html
    ns = rospy.get_namespace()
    rospy.init_node('dynamic_obstacles')
    startNode()