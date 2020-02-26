#!/usr/bin/env python3
# coding=utf-8
# author: thinkycx
# date: 2018-01-11
# modified by: Jesus Tordesillas, jtorde@mit.edu
# date: 2020-February



import math
import os
import sys

def create_session(session_name, commands):

    os.system("tmux new -d -s "+str(session_name)+" -x 300 -y 300")

    for i in range(len(commands)):
    	print('splitting ',i)
        os.system('tmux split-window ; tmux select-layout tiled')
   
    for i in range(len(commands)):
        os.system('tmux send-keys -t '+str(session_name)+':0.'+str(i) +' "'+ commands[i]+'" '+' C-m')

if __name__ == '__main__':
    commands = []
    num_of_agents=10;
    half_of_agents=num_of_agents/2.0
    dist_bet_groups=6.0
    for i in range(1,num_of_agents+1):
    	group = (i>=half_of_agents)

    	x= i if group == 0 else (i-half_of_agents)
    	y= dist_bet_groups*group   
    	z=0

        goal_x=half_of_agents-x
        goal_y=dist_bet_groups-y 
        goal_z=0

    	quad="SQ0" + str(i) + "s";
    	print ("quad= ",quad)
    	if(sys.argv[1]=="start"):
        	commands.append("roslaunch faster faster_specific.launch gazebo:=false quad:="+quad+" x:="+str(x)+" y:="+str(y)+" z:="+str(z))
    	if(sys.argv[1]=="send_goal"):
    		commands.append("rostopic pub /"+quad+"/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: "+str(goal_x)+", y: "+str(goal_y)+", z: "+str(goal_z)+"}, orientation: {w: 1.0}}}'")
    	if(sys.argv[1]=="faster"):
    		commands.append("roslaunch faster faster.launch quad:="+quad)
    print("len(commands)= " , len(commands))
    session_name=sys.argv[1] + "_session"
    os.system("tmux kill-session -t" + session_name)
    create_session(session_name, commands)
    os.system("tmux attach") #comment if you don't want to visualize all the terminals


#import libtmux
    # panes=win.list_panes()
    # print("panes.size=", len(panes))
    # for i in range(len(panes)):
    #     panes[i].send_keys(commands[i])


    # logging.info(panes)
    # logging.info(win)

            #win.select_layout(layout='tiled')
        #win.split_window()
        #win.cmd('select-layout tiled')  
        #win.select_layout(layout='tiled')
        #win.cmd('split-window', '-h')    

                #win.cmd('select-layout tiled')
        #os.system("tmux attach")
        #os.system("tmux select-layout tiled")

            #os.system("tmux attach")

    # win = session.new_window(attach=False, window_name="win")
    # win.select_layout(layout='tiled')
    #logging.info(commands)
    # pane_NUM = 3
    # WINDOW_NUM = int(math.ceil(len(commands)/4.0))  # in python3, we can use 4 also

    # server = libtmux.Server()
    # session = server.new_session(session_name)
    # panes = []