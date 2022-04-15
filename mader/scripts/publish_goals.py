import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

class TermGoalSender:

    def __init__(self):
        self.term_goal=PoseStamped();
        self.term_goal.header.frame_id='world';
        self.pubTermGoal = rospy.Publisher('/SQ01s/term_goal', PoseStamped, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(3.0), self.timerCB)


        self.all_goals=[];

        range_x=8
        range_y=8
        number=6

        for x_i in np.linspace(-range_x, range_x, num=number):
            for y_i in np.linspace(-range_y, range_y, num=number):
                self.all_goals.append(np.array([x_i,y_i,1.0]))

        self.index=0


    def timerCB(self, tmp):
        self.term_goal_pos=np.array([self.term_goal.pose.position.x,self.term_goal.pose.position.y,self.term_goal.pose.position.z])


        if(self.index==(len(self.all_goals))):
            quit()

        self.term_goal.pose.position.x=float(self.all_goals[self.index][0]);
        self.term_goal.pose.position.y=float(self.all_goals[self.index][1]);
        self.term_goal.pose.position.z=float(self.all_goals[self.index][2]);

        print(f"Publishing {self.all_goals[self.index]}")

        self.pubTermGoal.publish(self.term_goal) 


        self.index=self.index+1
       

def startNode():
    c = TermGoalSender()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('TermGoalSender')
    startNode()