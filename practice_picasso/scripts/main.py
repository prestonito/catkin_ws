#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import OccupancyGrid
import random
import actionlib

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path

class RandomPicasso:
    def __init__(self):
        self.map_subs = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.map_cb)
        self.map = None


        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.map:
                rospy.loginfo("we got the map")
                break
            rate.sleep()
        rospy.loginfo("we are done")

        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.visited_positions = []

        self.path_pub = rospy.Publisher("/traveled_path", Path, queue_size=1)

    def map_cb(self, msg):
        self.map = msg
        self.map_subs.unregister()

    def get_random_coordinate(self):
        found = False
        while not found:
            x = random.randint(0, self.map.info.width)
            y = random.randint(0, self.map.info.height)
            map_ij = self.map.data[y*self.map.info.width+x]
            if map_ij == 0:
                found = True
        
            return (x*self.map.info.resolution,y*self.map.info.resolution)

    def print_map(self):
        for i in range(self.map.info.height):
            for j in range(self.map.info.width):
                ai = i*self.map.info.width*j
                map_ij = self.map.data[ai]
                if map_ij == 100:
                    sys.stdout.write("X")
                elif map_ij == 0:
                    sys.stdout.write("_")
                else:
                    sys.stdout.write("0")
            sys.stdout.write("\n")


    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            random_coords = self.get_random_coordinate()
            rospy.loginfo("im going to" + str(random_coords))
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = random_coords[0]
            goal.target_pose.pose.position.y = random_coords[1]
            goal.target_pose.pose.orientation.w = 1
            self.move_base_client.send_goal(goal)
            self.visited_positions.append(goal.target_pose)
            path = Path()
            path.header.frame_id = "map"
            path.poses = self.visited_positions
            
            self.move_base_client.wait_for_result()
            self.path_pub.publish(path)
            rate.sleep()




if __name__ == "__main__":
    rospy.init_node("random_picasso_node", sys.argv)
    rp = RandomPicasso()
    rp.main_loop()
    rospy.loginfo("im done with the map")
    rospy.spin()
    