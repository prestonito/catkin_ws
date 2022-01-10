#! /usr/bin/env python

import rospy
#from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



def navigate():
    # Creates the SimpleActionClient, passing the type of the action

    #'/move_base' is topic and after comma is the type of action. we know this from rostopic info {NAME OF TOPIC}
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveBaseGoal();

    goal.target_pose.header.frame_id= 'map';
    goal.target_pose.header.stamp= rospy.Time.now()

    goal.target_pose.pose.orientation.w = 0.71;
    goal.target_pose.pose.orientation.z = -0.69;
    goal.target_pose.pose.orientation.

    goal.target_pose.pose.position.x = 6;
    goal.target_pose.pose.position.y = 8.4;



    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('robot_nav')
        result = navigate()
        #print("Result:", ', '.join([str(n) for n in result.sequence]))
        print("yay")
    except rospy.ROSInterruptException:
        print("program interrupted before completion")