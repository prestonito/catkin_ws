#!/usr/bin/env python
import rospy
import sys
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path

class PicassoNavigation:
    def __init__(self):
        self.machine_pose = rospy.Subscriber('/amcl_pose', String, self.amcl_cb))
        self.nav_msgs = rospy.Publisher('/nav_msgs/Path', Path, 
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.move_base_client = actionlib.SimplceActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()


    #generate random positions, store in a list, send goal to go to it, store amcl data
    def picasso_drawer(self):
        #self.amcl_pose_data = msg.data
        while True:
            target_coordinates = [(np.random.rand(0,13),(np.random.rand(0,10))]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = target_coordinates[0]
            goal.target_pose.pose.position.y = target_coordinates[1]
            goal.target_pose.pose.orientation.w = 1
            self.move_base_client.send_goal(goal)



            

    def generate_coordinates(self):
        target_coordinates = [(np.random.rand(0,13),(np.random.rand(0,10),1)]
       
        return target_coordinates

    def amcl_cb(self, msg):
        path.header = msg.header   #nav_msgs/Path is made up of Header and Pose
        pose = PoseStamped()            #initialises a pose object 
        pose.header = msg.header
        pose.pose = msg.pose.pose
        path.poses.append(pose)
        self.nav_msgs.publish(path)





if __name__ == "__main__":
    rospy.init_node("go_for_tea_sm", sys.argv)
    instance = PicassoNavigation()
    rospy.spin()