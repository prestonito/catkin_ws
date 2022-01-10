#!/usr/bin/env python
# coding=utf-8
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Here I define the coordinates of the rooms. I don't care about orientation in this case.
# The rooms are as follows (imagine below is the map, and the coordinates have the tag next to it)
# _______
# |A B C|
# |D E F|
# ¨¨¨¨¨¨¨
ROOMS = {'library': [1.74434804916, 8.51225471497],   # A
         'dining': [5.90392875671, 8.75152206421],    # B
         'living': [10.5052347183, 8.23617553711],    # C
         'bedroom': [1.79956388474, 2.71460771561],   # D
         'entrance': [5.92233419418, 3.02749752998],  # E
         'kitchen': [10.7260971069, 2.16245126724]}   # F


class SpeechControlAndRoomNavigation:
    def __init__(self):
        self.speech_subs = rospy.Subscriber('/speech_recognition/final_result', String, self.speech_received)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tts_pub = rospy.Publisher('/speech', String, queue_size=1)  # This one uses speech_database
        self.twist = Twist()
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.place = ''

    def speech_received(self, msg):
        self.recognized_speech = msg.data
        rospy.loginfo("I heard: " + self.recognized_speech)
        if 'forward' in self.recognized_speech:
            if self.move_base_client.get_state() == actionlib.GoalStatus.ACTIVE:
                self.tts_pub.publish('I am already going to the ' + self.place + ', tell me to stop to control me.')
            else:
                # move forward
                self.twist.linear.x = 0.5
                self.twist.angular.z = 0.0
                self.tts_pub.publish('I am moving forward')
        elif 'backward' in self.recognized_speech:
            if self.move_base_client.get_state() == actionlib.GoalStatus.ACTIVE:
                self.tts_pub.publish('I am already going to the ' + self.place + ', tell me to stop to control me.')
            else:
                # move backward
                self.twist.linear.x = -0.5
                self.twist.angular.z = 0.0
                self.tts_pub.publish('I am moving backward')
        elif 'left' in self.recognized_speech:
            if self.move_base_client.get_state() == actionlib.GoalStatus.ACTIVE:
                self.tts_pub.publish('I am already going to the ' + self.place + ', tell me to stop to control me.')
            else:
                # move left
                self.twist.angular.z = 1.0
                self.twist.angular.x = 0.0
                self.tts_pub.publish('I am turning left')
        elif 'right' in self.recognized_speech:
            if self.move_base_client.get_state() == actionlib.GoalStatus.ACTIVE:
                self.tts_pub.publish('I am already going to the ' + self.place + ', tell me to stop to control me.')
            else:
                # turn right
                self.twist.angular.z = -1.0
                self.twist.angular.x = 0.0
                self.tts_pub.publish('I am turning right')
        elif 'stop' in self.recognized_speech:
            self.move_base_client.cancel_goal()
            self.tts_pub.publish('I am stopping')
            self.twist = Twist()
        else:
            # Here I will split the recognized sentence, and look if any word matches our list of rooms. It may not be
            # the most efficient approach though.
            words = self.recognized_speech.split(' ')
            for w in words:
                if w in ROOMS:
                    coordinates = ROOMS[w]
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.pose.position.x = coordinates[0]
                    goal.target_pose.pose.position.y = coordinates[1]
                    goal.target_pose.pose.orientation.w = 1  # I don't care about the orientation here
                    self.place = w  # This will only be checked when there's an active goal, so the variable will have
                    # been set, so no need to reset it. Otherwise we would need to set it to a default value once we
                    # reached a goal
                    self.move_base_client.send_goal(goal)
                    self.tts_pub.publish('I am going to the ' + w)
                    # here I could self.move_base_client.wait_for_result(), but I this would stop the recognition service,
                    # so we wouldn't be able to get the callback called to stop the robot (or send a new goal).
                    # Instead, I am updating the main_loop to check if there's any active goal, and not publish the twist
                    # message if that is the case. Stop, therefore, will cancel an active goal
                    break

    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.move_base_client.get_state() != actionlib.GoalStatus.ACTIVE:
                self.cmd_pub.publish(self.twist)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('speech_control', sys.argv)
    speech_control = SpeechControlAndRoomNavigation()
    rospy.loginfo("I am ready")
    speech_control.main_loop()
    rospy.spin()
