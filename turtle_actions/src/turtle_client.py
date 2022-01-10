#! /usr/bin/env python

from __future__ import print_function
import actionlib
import rospy
from turtle_actions.msg import TurtleNavigationAction, TurtleNavigationGoal
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose



def draw_square():
    poses = [Pose2D(3.5, 7.5, 0), Pose2D(7.5, 7.5, 0), Pose2D(7.5, 3.5, 0), Pose2D(3.5, 3.5, 0)]

    client = actionlib.SimpleActionClient('move_turtle', TurtleNavigationAction)
    client.wait_for_server()

    for next_pose in poses:
        goal = TurtleNavigationGoal()
        goal.target_pose = next_pose

        client.send_goal(goal)

        client.wait_for_result()


if __name__ == '__main__':
    try:
        rospy.init_node('square_node')
        draw_square()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
