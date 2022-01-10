#! /usr/bin/env python

import rospy
# from __future__ import print_function



# Brings in the SimpleActionClient
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal







def navigate(target):
    # Creates the SimpleActionClient, passing the type of the action

    # 'green' is topic and after comma is the type of action. we know this from rostopic info {NAME OF TOPIC}
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveBaseGoal();

    goal.target_pose.header.frame_id = 'map';
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.orientation.w = compute_quaternion(target[2])[3];
    goal.target_pose.pose.orientation.z = compute_quaternion(target[2])[2];
    goal.target_pose.pose.orientation.

    goal.target_pose.pose.position.x = target[0];
    goal.target_pose.pose.position.y = target[1];

    # Sends the goal to the action server.
    client.send_goal(goal)

    #sees if its done
    print("goal passed to server")

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


def compute_quaternion(orientation):

    theta = np.deg2rad(orientation)

    z = np.sin(theta/2)
    w = np.cos(theta/2)



    q = [0, 0, z, w] #in quaternion, since we are only operating in 2D, first two terms are always zero

    print("q is",q)

    return q

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('robot_nav')

        goal_pos = np.zeros(3)
        goal_pos[0] = float(input("Goal x-coordinate:"))
        goal_pos[1] = float(input("Goal y-coordinate:"))
        goal_pos[2] = float(input("Goal orientation angle in degrees:"))

        result = navigate(goal_pos)

        print("yay")
    except rospy.ROSInterruptException:
        print("program interrupted before completion")