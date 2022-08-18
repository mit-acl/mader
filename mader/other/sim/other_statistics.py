#!/usr/bin/python

# Jesus Tordesillas Torres, jtorde@mit.edu, April 2020

#Instructions:
#roscore
#rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 map world 10
#python this_file.py

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import rospy
import numpy as np
import rosbag
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

import sys

from tf_bag import BagTfTransformer
import numpy as np
from colorama import init, Fore, Back, Style

from termcolor import colored

import re
import glob


if (len(sys.argv) <=1):
    print "Usage is python this_file.py name_of_bag.bag "
    print "or"
    print 'Usage is python this_file.py "2020_*.bag" (do not forget the "")'
    sys.exit(1)


#rospy.init_node('talker', anonymous=True)

list_of_bags=glob.glob(sys.argv[1]);

# print list_of_bags

for name_bag in list_of_bags:

    print Fore.BLUE + "====================================="
    print Fore.BLUE + "====================================="
    

    print Fore.BLUE + "Reading ",name_bag

    print Style.RESET_ALL

    bag = rosbag.Bag(name_bag)
    all_topics = bag.get_type_and_topic_info()[1].keys()
    regex = re.compile('/SQ.+s/goal') # .+ is the wildcard
    res = [i for i in all_topics if re.match(regex, i)] 
    
    num_of_agents=len(res); 


    print "Detected ", num_of_agents, " agents in the bag"

    drone_radius=0.05;

    topics=[]
    agents_names=[]
    for i in range(1,num_of_agents+1):
        topics.append('/SQ0'+str(i)+'s/goal')
        agents_names.append('SQ0'+str(i)+'s')



    distances=[];

    x_old=0.0;
    y_old=0.0;
    z_old=0.0;

    x_old_marker=0.0;
    y_old_marker=0.0;
    z_old_marker=0.0;

    marker_array_pub_= rospy.Publisher('/actual_trajectories', MarkerArray, queue_size=10)



    # final_time_bag=bag.get_end_time()
    # t_go_bag=bag.get_start_time()
    # total_time_bag=final_time_bag-t_go_bag;

    ####################3
    #Time when the first terminal goal is sent to the UAVs
    t_go=float("inf")
    t_1start=float("inf")
    t_1end=float("inf")
    t_laststart=0.0
    for index_agent in range(0, num_of_agents):
        for topic, msg, t in bag.read_messages(topics='/'+agents_names[index_agent]+'/term_goal'):
            t_go=min(t_go, msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9)

    #Time when the last UAV reaches the goal terminal goal is sent to the UAVs  
    epsilon=1e-7 
    final_time= 0
    num_of_stops=0.0
    for index_agent in range(0, num_of_agents):
        stopped=True; #Every agent starts stopped
        have_started_flying=False;
        #print "=========================="
        #print "Agent= ", index_agent
        for topic, msg, t in bag.read_messages(topics='/'+agents_names[index_agent]+'/goal'):
            final_time=max(final_time, msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9)
            vel=np.linalg.norm(np.array([msg.vel.x, msg.vel.y, msg.vel.z]));

            if (vel<epsilon and stopped==False):
                stopped=True
                num_of_stops=num_of_stops+1;
                #print msg;

            if (vel>epsilon and stopped==True and have_started_flying==False):
                have_started_flying=True
                time_now=msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
                t_1start=min(t_1start, time_now)
                t_laststart=max(t_laststart, time_now)


            if (vel>epsilon and stopped==True):
                stopped=False
                        
            last_vel=vel;
        #Out of the inner loop now
        t_1end=min(t_1end, msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9)


        if(last_vel<epsilon and stopped==True):#Don't count the stop at the end of the simulation
            num_of_stops=num_of_stops-1;
            #print "Discounting here"

    t_1start=t_1start-t_go;
    t_1end=t_1end-t_go;
    t_laststart=t_laststart-t_go;






    total_time=final_time-t_go;
    #################

    # print "t_go= ", t_go
    # print "final_time= ", final_time
    # print "__________________________"

    # print "__________________________"




    markerArray = MarkerArray()

    j=0;

    skip=20;



    for index_agent in range(0, num_of_agents):
       # print("Using topic= ",topics[index_agent])
        total_dist=0.0;
        index_msg=0
        j=0
        topic_name=topics[index_agent];

        for topic, msg, t in bag.read_messages(topics=topic_name):

            vx=msg.vel.x
            vy=msg.vel.y
            vz=msg.vel.z
            
            x=msg.pos.x
            y=msg.pos.y
            z=msg.pos.z          

            if(j%skip==0 and j>5):
                myMarker = Marker()
                myMarker.header.frame_id = "world"
                myMarker.header.seq = i
                #myMarker.header.stamp    = rospy.get_rostime()
                #myMarker.ns = "robot"
                myMarker.id = j
                myMarker.ns = "agent"+str(index_agent+1)
                myMarker.type = myMarker.ARROW # sphere
                myMarker.points=[0,0] #Declare the list
                myMarker.points[0]=Point(x_old_marker,y_old_marker,z_old_marker)
                myMarker.points[1]=Point(x,y,z)
                myMarker.action = myMarker.ADD

                # if(abs(x-x_old)>0.5 or abs(y-y_old)>0.5):
                #     continue

                time_received=t.secs + t.nsecs*1e-9;
                time_relative=(time_received-t_go)/(0.85*total_time);
                color=cm.jet(int(time_relative*256));
                myMarker.color=ColorRGBA(color[0], color[1], color[2], color[3])
                myMarker.scale.x=0.15
                myMarker.scale.y=0.001
                myMarker.pose.orientation.x=myMarker.pose.orientation.y=myMarker.pose.orientation.z=0.0;
                myMarker.pose.orientation.w=1.0;
                #if(abs(x-x_old_marker)<0.5 and abs(y-y_old_marker)<0.5): #TODO
                markerArray.markers.append(myMarker) 
                x_old_marker=x
                y_old_marker=y
                z_old_marker=z

            if(j != 0):
                delta_x=(x-x_old);
                delta_y=(y-y_old);
                delta_z=(z-z_old);
                total_dist=total_dist+math.sqrt(delta_x*delta_x +delta_y*delta_y+ delta_z*delta_z)
                #print("total_dist= ",total_dist)
            if(j==0):
                x_old_marker=x
                y_old_marker=y
                z_old_marker=z

            x_old=x
            y_old=y
            z_old=z

            j=j+1;


            index_msg=index_msg+1

        if(index_msg!=0):
            #print ("Total distance agent "+str(index_agent+1)+" =",total_dist) 
            distances.append(total_dist);






    times_discret = np.linspace(t_go, final_time, 2000)


    print "Constructing bag transformer..."
    bag_transformer = BagTfTransformer(bag)
    print "Constructed, checking now min distances"


    summary_crashes=[]
    all_min_distances=[]

    for i in range(0,num_of_agents):
        agent_i=agents_names[i];
        printed=False;
        for j in range(i+1,num_of_agents):
            min_distance= float("inf")
            
            agent_j=agents_names[j];
            # print "*******************",agent_i, "-->", agent_j

            for ii in range(len(times_discret)-1):
                time=times_discret[ii];

                try:                             
                    translation1, quaternion = bag_transformer.lookupTransform("vicon", agent_i, rospy.Time.from_sec(time))
                    translation2, quaternion = bag_transformer.lookupTransform("vicon", agent_j, rospy.Time.from_sec(time))
                    dist=np.linalg.norm(np.array(translation1)-np.array(translation2));
                    min_distance=min(min_distance, dist)

                except Exception as e: 
                    pass
                    # print "Exception:", str(e)
            all_min_distances.append(min_distance);
            sentence=agent_i+ '-->'+ agent_j + ' min distance= '+ str(min_distance)
            if(min_distance<2*drone_radius):
                print Fore.RED + sentence + Style.RESET_ALL
                summary_crashes.append(sentence)
            else:
                #print Fore.GREEN + sentence + Style.RESET_ALL
                pass

    if len(summary_crashes)==0:
        print Fore.GREEN + "NO CRASHES"
    else:
        print Fore.RED + "CRASHES: "
        for crash_i in summary_crashes:
            print Fore.RED + crash_i
    print Style.RESET_ALL 

    safety_margin_ratio=min(all_min_distances)/(2*drone_radius); #Should be >=1 to ensure safety
    print "Num of agents: ", num_of_agents
    print "Safety Margin Ratio: ", safety_margin_ratio
    print "Sum dist all the agents: ", sum(distances)
    print "Total_time (seconds): ", total_time
    print "Total number of Stops: ",num_of_stops
    print "Total number of stops per agent: ",num_of_stops/float(num_of_agents)
    print "t_1start: ", t_1start
    print "t_laststart: ", t_laststart
    print "t_1end: ", t_1end
    print "total_time: ", total_time

    print "(Using radius drone= ", drone_radius, ")"
    #print("Publishing Array:")

    #marker_array_pub_.publish(markerArray)
    rospy.sleep(4.0)


    bag.close()