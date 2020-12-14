#!/usr/bin/env python

#Jesus Tordesillas Torres, December 2019

#This files plots in gazebo with the position and orientation of the drone according to the desired position and acceleration specified in the goal topic

import roslib
import rospy
import math
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
import numpy as np
from numpy import linalg as LA

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply

from visualization_msgs.msg import Marker

import tf



class FakeSim:

    def __init__(self):
        self.state=State()

        self.state.pos.x = rospy.get_param('~x', 0.0);
        self.state.pos.y = rospy.get_param('~y', 0.0);
        self.state.pos.z = rospy.get_param('~z', 0.0);
        yaw = rospy.get_param('~yaw', 0.0);

        pitch=0.0;
        roll=0.0;
        quat = quaternion_from_euler(yaw, pitch, roll, 'rzyx')

        self.state.quat.x = quat[0]
        self.state.quat.y = quat[1]
        self.state.quat.z = quat[2]
        self.state.quat.w = quat[3]

        self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.pubMarkerDrone = rospy.Publisher('marker', Marker, queue_size=1, latch=True)
        self.pubState = rospy.Publisher('state', State, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.pubTF)
        name = rospy.get_namespace()
        self.name = name[1:-1]
        
        rospy.sleep(1.0)

        self.state.header.frame_id="world"
        self.pubState.publish(self.state)  

        pose=Pose()
        pose.position.x=self.state.pos.x;
        pose.position.y=self.state.pos.y;
        pose.position.z=self.state.pos.z;
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        self.pubMarkerDrone.publish(self.getDroneMarker(pose));




    def goalCB(self, data):


        state = State()
        gazebo_state = ModelState()
        gazebo_state.model_name = self.name
        axis_z=[0,0,1]

        accel=[data.a.x, data.a.y, data.a.z + 9.81];

        gazebo_state.pose.position.x = data.p.x
        gazebo_state.pose.position.y = data.p.y
        gazebo_state.pose.position.z = data.p.z


        drone_quaternion_with_yaw=[];

        if(LA.norm(accel)>0.000001 and LA.norm(np.cross(accel, axis_z))>0.0000001):
          norm_accel=LA.norm(accel)
          accel=accel/norm_accel
          axis=np.cross(accel, axis_z);

          dot=np.dot(accel,axis_z)
          angle=math.acos(dot)        
          drone_quaternion = quaternion_about_axis(-angle, axis)# Quaternion(axis=axis, angle=-angle)
          
          #Use the yaw from goal
          drone_quaternion_with_yaw=quaternion_multiply(drone_quaternion,quaternion_from_euler(data.yaw, 0.0, 0.0, 'rzyx'))

        else: #Take only the yaw angle
          drone_quaternion_with_yaw = quaternion_from_euler(data.yaw, 0.0, 0.0, 'rzyx')

        gazebo_state.pose.orientation.x = drone_quaternion_with_yaw[0]
        gazebo_state.pose.orientation.y = drone_quaternion_with_yaw[1]
        gazebo_state.pose.orientation.z = drone_quaternion_with_yaw[2]
        gazebo_state.pose.orientation.w = drone_quaternion_with_yaw[3]



        #self.gazebo_state.twist = data.twist

        ## HACK TO NOT USE GAZEBO
        gazebo_state.reference_frame = "world" 
        self.pubGazeboState.publish(gazebo_state)  
        ## END OF HACK TO NOT USE GAZEBO



        self.state.header.frame_id="world"
        self.state.pos=data.p
        self.state.vel=data.v
        self.state.quat=gazebo_state.pose.orientation
        self.pubState.publish(self.state)  
        # print("State after:")
        # print(self.state.quat)

        self.pubMarkerDrone.publish(self.getDroneMarker(gazebo_state.pose));

    def pubTF(self, timer):
        br = tf.TransformBroadcaster()
        br.sendTransform((self.state.pos.x, self.state.pos.y, self.state.pos.z),
                         (self.state.quat.x, self.state.quat.y, self.state.quat.z, self.state.quat.w),
                         rospy.Time.now(),
                         self.name,
                         "vicon")

    def getDroneMarker(self, pose):
        marker=Marker();
        marker.id=1;
        marker.ns="mesh_"+self.name;
        marker.header.frame_id="world"
        marker.type=marker.MESH_RESOURCE;
        marker.action=marker.ADD;

        marker.pose=pose
        marker.lifetime = rospy.Duration.from_sec(0.0);
        marker.mesh_use_embedded_materials=True
        marker.mesh_resource="package://acl_sim/meshes/quadrotor/quadrotor.dae"
        marker.scale.x=1.0;
        marker.scale.y=1.0;
        marker.scale.z=1.0;
        return marker 



             

def startNode():
    c = FakeSim()
    rospy.Subscriber("goal", Goal, c.goalCB)

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
            print "Starting perfect tracker node for: " + ns
            startNode()
    except rospy.ROSInterruptException:
        pass
