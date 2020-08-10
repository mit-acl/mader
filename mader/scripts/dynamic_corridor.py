#!/usr/bin/env python

#Jesus Tordesillas Torres, December 2019

#This files plots in gazebo with the position and orientation of the drone according to the desired position and acceleration specified in the goal topic

import random
import roslib
import rospy
import math
from mader_msgs.msg import DynTraj
from snapstack_msgs.msg import QuadGoal, State
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

from pyquaternion import Quaternion
import tf

from math import sin, cos, tan


import copy 
import sys

color_static=ColorRGBA(r=0,g=0,b=1,a=1);
color_dynamic=ColorRGBA(r=1,g=0,b=0,a=1);

class MovingCircle:
    def __init__(self):
        self.radius=1.0;
        self.num_of_objects=3;
        self.scale=1.0;
        self.slower_min=1
        self.slower_max= 1.5

class MovingCorridor:
    def __init__(self, total_num_obs):
        print(total_num_obs)
        self.num_of_dyn_objects=int(0.65*total_num_obs);
        self.num_of_stat_objects=total_num_obs-self.num_of_dyn_objects; #They are actually dynamic obstacles
        self.x_min= 2.0
        self.x_max= 75.0
        self.y_min= -3.0 
        self.y_max= 3.0
        # self.x_min= 0.0
        # self.x_max= 0.0
        # self.y_min= 0.0 
        # self.y_max= 0.0
        self.z_min= 1.0 
        self.z_max= 2.0
        self.scale=1.0;
        self.slower_min=1.1
        self.slower_max= 1.1
        self.bbox_dynamic=[0.8, 0.8, 0.8] #[0.6, 0.6, 0.6]
        self.bbox_static_vert=[0.4, 0.4, 4]
        self.bbox_static_horiz=[0.4, 8, 0.4]
        self.percentage_vert=0.35;



class FakeSim:

    def __init__(self, total_num_obs):
        self.state=State()

        name = rospy.get_namespace()
        self.name = name[1:-1]

       #self.num_of_objects = 0;

        world_type="MovingCorridor"

        if(world_type=="MovingCorridor"):
            self.world=MovingCorridor(total_num_obs)
        if(world_type=="MovingCircle"):
            self.world=MovingCircle()
   
        available_meshes_static=["package://mader/meshes/ConcreteDamage01b/model3.dae", "package://mader/meshes/ConcreteDamage01b/model2.dae"]
        available_meshes_dynamic=["package://mader/meshes/ConcreteDamage01b/model4.dae"]
        # available_meshes=["package://mader/meshes/ConcreteDamage01b/model3.dae"]

        self.x_all=[];
        self.y_all=[];
        self.z_all=[];
        self.offset_all=[];
        self.slower=[];
        self.meshes=[];
        self.type=[];#"dynamic" or "static"
        self.bboxes=[]; 
        for i in range(self.world.num_of_dyn_objects):          
            self.x_all.append(random.uniform(self.world.x_min, self.world.x_max));
            self.y_all.append(random.uniform(self.world.y_min, self.world.y_max));
            self.z_all.append(random.uniform(self.world.z_min, self.world.z_max));
            self.offset_all.append(random.uniform(-2*math.pi, 2*math.pi));
            self.slower.append(random.uniform(self.world.slower_min, self.world.slower_max));
            self.type.append("dynamic")
            self.meshes.append(random.choice(available_meshes_dynamic));
            self.bboxes.append(self.world.bbox_dynamic);

        for i in range(self.world.num_of_stat_objects):
            bbox_i=[]; 
            if(i<self.world.percentage_vert*self.world.num_of_stat_objects):
                bbox_i=self.world.bbox_static_vert;
                self.z_all.append(bbox_i[2]/2.0);
                self.type.append("static_vert")
            else:
                bbox_i=self.world.bbox_static_horiz;
                self.z_all.append(random.uniform(0.0, 3.0));
                self.type.append("static_horiz")


            self.x_all.append(random.uniform(self.world.x_min-self.world.scale, self.world.x_max+self.world.scale));
            self.y_all.append(random.uniform(self.world.y_min-self.world.scale, self.world.y_max+self.world.scale));
            
            self.offset_all.append(random.uniform(-2*math.pi, 2*math.pi));
            self.slower.append(random.uniform(self.world.slower_min, self.world.slower_max));
            # self.type.append("static")
            self.meshes.append(random.choice(available_meshes_static));
            self.bboxes.append(bbox_i)


        self.pubTraj = rospy.Publisher('/trajs', DynTraj, queue_size=1, latch=True)
        self.pubShapes_static = rospy.Publisher('/shapes_static', Marker, queue_size=1, latch=True)
        self.pubShapes_static_mesh = rospy.Publisher('/shapes_static_mesh', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic_mesh = rospy.Publisher('/shapes_dynamic_mesh', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic = rospy.Publisher('/shapes_dynamic', Marker, queue_size=1, latch=True)
        self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        self.already_published_static_shapes=False;

        rospy.sleep(0.5)


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


        ###################3
        marker_array_static_mesh=MarkerArray();
        marker_array_dynamic_mesh=MarkerArray();

        for i in range(self.world.num_of_dyn_objects + self.world.num_of_stat_objects):
            t_ros=rospy.Time.now()
            t=rospy.get_time(); #Same as before, but it's float

            dynamic_trajectory_msg=DynTraj(); 

            bbox_i=self.bboxes[i];
            s=self.world.scale;
            if(self.type[i]=="dynamic"):

              [x_string, y_string, z_string] = self.trefoil(self.x_all[i], self.y_all[i], self.z_all[i], s,s,s, self.offset_all[i], self.slower[i]) 
              # print("self.bboxes[i]= ", self.bboxes[i])
              dynamic_trajectory_msg.bbox = bbox_i;
              marker_dynamic.scale.x=bbox_i[0]
              marker_dynamic.scale.y=bbox_i[1]
              marker_dynamic.scale.z=bbox_i[2]
            else:
              # [x_string, y_string, z_string] = self.static(self.x_all[i], self.y_all[i], self.z_all[i]);
              dynamic_trajectory_msg.bbox = bbox_i;
              marker_static.scale.x=bbox_i[0]
              marker_static.scale.y=bbox_i[1]
              marker_static.scale.z=bbox_i[2]
              if(self.type[i]=="static_vert"):
                [x_string, y_string, z_string] = self.wave_in_z(self.x_all[i], self.y_all[i], self.z_all[i], s, self.offset_all[i], 1.0)
              else:
                [x_string, y_string, z_string] = self.wave_in_z(self.x_all[i], self.y_all[i], self.z_all[i], s, self.offset_all[i], 1.0)




            x = eval(x_string)
            y = eval(y_string)
            z = eval(z_string)

            dynamic_trajectory_msg.is_agent=False;
            dynamic_trajectory_msg.header.stamp= t_ros;
            dynamic_trajectory_msg.function = [x_string, y_string, z_string]
            dynamic_trajectory_msg.pos.x=x #Current position
            dynamic_trajectory_msg.pos.y=y #Current position
            dynamic_trajectory_msg.pos.z=z #Current position

            dynamic_trajectory_msg.id = 4000+ i #Current id 4000 to avoid interference with ids from agents #TODO

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
            marker.id=i;
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
            marker.mesh_resource=self.meshes[i]

            if(self.type[i]=="dynamic"):
                marker_dynamic.points.append(point);

                marker.scale.x=bbox_i[0];
                marker.scale.y=bbox_i[1];
                marker.scale.z=bbox_i[2];

                marker_array_dynamic_mesh.markers.append(marker);


            if(self.type[i]=="static_vert" or self.type[i]=="static_horiz"):

                marker.scale.x=bbox_i[0];
                marker.scale.y=bbox_i[1];
                marker.scale.z=bbox_i[2];
                
                marker_array_static_mesh.markers.append(marker);

                ##########################

                marker_static.points.append(point);

        self.pubShapes_dynamic_mesh.publish(marker_array_dynamic_mesh)
        self.pubShapes_dynamic.publish(marker_dynamic)


        # if(self.already_published_static_shapes==False):

        self.pubShapes_static_mesh.publish(marker_array_static_mesh)
        self.pubShapes_static.publish(marker_static)

        # self.already_published_static_shapes=True;



    def static(self,x,y,z):
        return [str(x), str(y), str(z)]

    # Trefoil knot, https://en.wikipedia.org/wiki/Trefoil_knot
    def trefoil(self,x,y,z,scale_x, scale_y, scale_z, offset, slower):

        #slower=1.0; #The higher, the slower the obstacles move" 
        tt='t/' + str(slower)+'+';

        x_string=str(scale_x)+'*(sin('+tt +str(offset)+') + 2 * sin(2 * '+tt +str(offset)+'))' +'+' + str(x); #'2*sin(t)' 
        y_string=str(scale_y)+'*(cos('+tt +str(offset)+') - 2 * cos(2 * '+tt +str(offset)+'))' +'+' + str(y); #'2*cos(t)' 
        z_string=str(scale_z)+'*(-sin(3 * '+tt +str(offset)+'))' + '+' + str(z);                               #'1.0'        

        # x_string='sin('+tt +str(offset)+')';
        # y_string='cos('+tt +str(offset)+')';
        # z_string='1'

        return [x_string, y_string, z_string]

    def wave_in_z(self,x,y,z,scale, offset, slower):

        tt='t/' + str(slower)+'+';

        x_string=str(x);
        y_string=str(y)
        z_string=str(scale)+'*(-sin( '+tt +str(offset)+'))' + '+' + str(z);                     

        return [x_string, y_string, z_string]


             

def startNode(total_num_obs):
    c = FakeSim(total_num_obs)
    rospy.Timer(rospy.Duration(0.01), c.pubTF)
    rospy.spin()

if __name__ == '__main__':

    # I think I should use https://docs.python.org/3.3/library/argparse.html
    print("********************************")
    print(sys.argv)
    if(len(sys.argv)<=1):
        # print("Usage: python dynamic_corridor.py [Num_of_obstacles]")
        total_num_obs=140; 
    else:
        total_num_obs=int(sys.argv[1])

    # print("sys.argv[1]= ", sys.argv[1])
    # total_num_obs=140
    ns = rospy.get_namespace()
    try:
        rospy.init_node('dynamic_obstacles')
        startNode(total_num_obs)
    except rospy.ROSInterruptException:
        pass


            # self.x_all.append(random.random());
            # self.y_all.append(4*random.random());
            # self.z_all.append(2);
            # self.x_all.append(50*random.random());
            # self.y_all.append(1*random.random());
            # self.z_all.append(2);

        # self.state.quat.x = 0
        # self.state.quat.y = 0
        # self.state.quat.z = 0
        # self.state.quat.w = 1

        # self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        # self.state.header.frame_id="world"
        # self.pubState.publish(self.state)  