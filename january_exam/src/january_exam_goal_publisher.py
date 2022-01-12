#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Pose2D


class GoalPublisher:
    def __init__(self):
        self.goal_publisher = rospy.Publisher("/exam_goal", Pose2D, queue_size=1)
        self.goal_pose = Pose2D(0.927, 4.95, 1.57)

    def main_loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.goal_publisher.publish(self.goal_pose)
            #rospy.loginfo('i published') #testing to make sure it's publishing
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("exam_goal_publisher", sys.argv)
    gp = GoalPublisher()
    try:
        gp.main_loop()
    except rospy.ROSInterruptException:
        pass
