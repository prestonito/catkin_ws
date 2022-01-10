#! /usr/bin/env python

import math
import angles
import numpy as np
import actionlib
import rospy
from turtle_actions.msg import TurtleNavigationAction, TurtleNavigationFeedback, TurtleNavigationGoal
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



class TurtleServer:

    def __init__(self):
        self._as = actionlib.SimpleActionServer('move_turtle', TurtleNavigationAction, execute_cb=self.go_to_goal, auto_start=False)
        self._as.start()
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()


    def go_to_goal(self,goal):

        d = np.zeros(2)
        p = np.zeros(2)
        g = np.zeros(2)

        g[0] = self.pose.x
        p[1] = self.pose.y

        g[0] = goal.target_pose.x
        g[1] = goal.target_pose.y

        d = g - p

        distance = np.linalg.norm(d)
        rate = rospy.Rate(10)

        threshold = 0.1

        while not rospy.is_shutdown() and self._as.is_active() and distance > threshold:
            angle = math.atan2(d[1], d[0])
            angle_difference = angles.shortest_angular_distance(self.pose.theta, angle)

            twist = Twist()

            if angle_difference > 0.0872 or angle_difference < -0.0872:
                twist.angular.z = angle_difference
            else:
                twist.angular.z = angle_difference
                if distance > threshold * 2:
                    twist.linear.x = 2
                else:
                    twist.linear.x = 0.1

            self.velocity_publisher.publish(twist)

            feedback = TurtleNavigationFeedback()
            feedback.distance = 0.5 * distance
            self._as.publish_feedback(feedback)

            rate.sleep()

            p[0] = self.pose.x
            p[1] = self.pose.y

            g[0] = goal.target_pose.x
            g[1] = goal.target_pose.y

            d = g - p

            distance = np.linalg.norm(d)

        if distance <= threshold:
            self._as.set_succeeded()
        else:
            self._as.set_preempted()

    def update_pose(self, data):
        self.pose = data



if __name__ == '__main__':
    rospy.init_node('turtle_navigation')
    server = TurtleServer()
    rospy.spin()
