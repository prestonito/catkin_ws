#!/usr/bin/env python

import rospy 
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from itr_tutorials.srv import Str2Coord
from itr_tutorials.msg import GetTeaAction


class GoForTeaNode:
    def __init__(self):
        
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) #making an actionlib client
        self.move_base_client.wait_for_server() #waits for server to prevent errors

        self.coord_srv = rospy.ServiceProxy('/str_to_coord', Str2Coord) #basically a client to the service we created (string to coord)

        self.kitchen_coords = self.coord_srv('kitchen').coordinates #gets the kitchen coordinates from the service we created
        
        self.go_tea_action = actionlib.SimpleActionServer('/get_me_tea', GetTeaAction, self.go_get_tea) #created actionlib server with our own defined message

        rospy.loginfo('I am ready to get tea!!!!')

    def go_get_tea(self):
        place = goal.destination #the request part of the service def file = goal part of action def file

        kitchen_goal = MoveBaseGoal() #makes "kitchen_goal" the MoveBaseGoal. using rosmsg show MoveBaseGoal,
                                        #we can see that its a geometry_msgs/PoseStamped and a target_pose
        kitchen_goal.target_pose = self.kitchen_coords #assigning the target_pose (from MoveBaseGoal) to kitchen coords from the service

        self.move_base_client.send_goal(kitchen_goal) #sends goal
        self.move_base_client.wait_for_result


if __name__ == '__main__':
    rospy.init_node('go_for_tea_node', sys.argv)
    instance = GoForTeaNode()
    rospy.spin()