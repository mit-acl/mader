#!/usr/bin/env python

#Jesus Tordesillas Torres, December 2019

import roslib, rospy, math, tf
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
import numpy as np
from numpy import linalg as LA
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply
from visualization_msgs.msg import Marker



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


    def goalCB(self, data):


        state = State()

        axis_z=[0,0,1]

        accel=[data.a.x, data.a.y, data.a.z + 9.81];

        drone_quaternion_with_yaw=[];

        if(LA.norm(accel)>1e-6 and LA.norm(np.cross(accel, axis_z))>1e-6): #TODO
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



        self.state.header.frame_id="world"
        self.state.pos=data.p
        self.state.vel=data.v
        self.state.quat.x = drone_quaternion_with_yaw[0]
        self.state.quat.y = drone_quaternion_with_yaw[1]
        self.state.quat.z = drone_quaternion_with_yaw[2]
        self.state.quat.w = drone_quaternion_with_yaw[3]

        self.pubState.publish(self.state)  

    def pubTF(self, timer):
        br = tf.TransformBroadcaster()
        br.sendTransform((self.state.pos.x, self.state.pos.y, self.state.pos.z),
                         (self.state.quat.x, self.state.quat.y, self.state.quat.z, self.state.quat.w),
                         rospy.Time.now(),
                         self.name,
                         "vicon")

            

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
            rospy.logfatal("This is typically accomplished in a launch file.")
        else:
            print "Starting perfect tracker node for: " + ns
            startNode()
    except rospy.ROSInterruptException:
        pass
