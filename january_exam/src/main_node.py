#!/usr/bin/env python
# Student name: Preston Ito
import rospy
import sys
import math
import actionlib
from geometry_msgs.msg import Pose2D
from january_exam.msg import ExamInfo
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class ITRExamNode:
    def __init__(self):
        # Put your code here (remove the pass keyword)
        # initialize here your variables to be used throughout the class

        self.exam_goal_subs = rospy.Subscriber('/exam_goal', Pose2D, self.send_goal)
        self.exam_pub = rospy.Publisher('/exam_updates', ExamInfo, queue_size=1)

        #sending goal using MoveBaseGoal
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo('i succseffully made movebaseclient')
        #used to get the robot's current position
        self.pose = Pose2D()




    def send_goal(self, msg):

        

        #creates and sends goal to move_base_client
        rospy.loginfo('im about to send goal')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'

        x_coord = msg.x
        y_coord = msg.y
        theta = msg.theta
        goal.target_pose.pose.position.x = x_coord 
        goal.target_pose.pose.position.y = y_coord 
        quaternion_w = math.cos((theta)/2)

        goal.target_pose.pose.orientation.w = quaternion_w
        self.move_base_client.send_goal(goal)

        rospy.loginfo('going to goal')
        self.exam_pub.publish('moving towards', msg)
        #waits until it reaches position to publish message on /exam_updates
        self.move_base_client.wait_for_result()
        rospy.loginfo('yay i went to goal')
        self.exam_pub.publish('I am currently at position: ', msg)


    @staticmethod
    def distance(a, b):
        '''
        Returns the Euclidean distance from point a to b.

            Parameters:
                a (tuple): 2D coordinates (x, y) of one point in the map.
                b (tuple): 2D coordinates (x, y) of one point in the map.

            Returns:
                float: The distance from point a to point b in the map.

            Examples:
                x = distance((0.5, 0.25), (5.689, 2.358)) -> 5.600837883745609

                a = (5.896, 9.258)
                b = (3.689, 7.841)
                x = distance(a, b) -> 2.622734832193296
        '''
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


if __name__ == "__main__":
    # Put your code here for the main method
    rospy.init_node('exam_node', sys.argv)
    instance = ITRExamNode()
    rospy.spin()
