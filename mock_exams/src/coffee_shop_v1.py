#!/usr/bin/env python
# coding=utf-8
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
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
         'kitchen': [10.7260971069, 2.16245126724],   # F
         'rest': [6.1, 6]}



DRINKS = {'tea': [6.5, 9.5],   
         'coffee': [11.6, 1.9],    
         'hot_chocolate': [4.8, 5.1],    
         'donut': [7.5, 4.38],  
         'pretzel': [0.9, 3.8],  
         'water': [9.1, 4.2]}   



class SpeechControlAndRoomNavigation:
    def __init__(self):
        self.speech_subs = rospy.Subscriber('/speech_recognition/final_result', String, self.speech_received)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tts_pub = rospy.Publisher('/speech', String, queue_size=1)  # This one uses speech_database
        self.twist = Twist()  #initializing the robots movement for move forward, backward, turn, etc.
        self.pose = Pose()
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.place = ''

    def speech_received(self, msg):
        self.recognized_speech = msg.data
        rospy.loginfo("I heard: " + self.recognized_speech)
        
        
        if self.recognized_speech != None:
        
            # creates list
            drinks_list = []
            #rooms_list = []
            #rooms = ''
            words = self.recognized_speech.split(' ')
            for w in words:
                if w in DRINKS:
                    #store drinks in a list
                    drinks_list.append(w)
                    continue
                elif w in ROOMS:
                    #option 1
                    #rooms_list.append(w)
                    #option 2
                    room_name = w
                    continue
                else:
                    rospy.loginfo('idk what you want from me')

            for d in drinks_list:
                coordinates = DRINKS[d]
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = coordinates[0]
                goal.target_pose.pose.position.y = coordinates[1]
                goal.target_pose.pose.orientation.w = 1  # I don't care about the orientation here
                self.move_base_client.send_goal(goal)
                self.tts_pub.publish('I am going to get the ' + d)
                rospy.loginfo('i am going to get the ' + d)

                self.move_base_client.wait_for_result()
                rospy.loginfo('i got the ' + d)

               # self.pose = p


                #while self.pose != goal.target_pose.pose:
                 #   rospy.loginfo('i am currently at ' self.pose + ' and trying to get to ' + goal.target_pose.pose )
                  #  rospy.loginfo('i am going to get the ' + d)

            
            #once finished collecting drinks, run this code below to deliver
            room_coordinates = ROOMS[room_name]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = room_coordinates[0]
            goal.target_pose.pose.position.y = room_coordinates[1]
            goal.target_pose.pose.orientation.w = 1  # I don't care about the orientation here
            self.move_base_client.send_goal(goal)
            self.tts_pub.publish('I am going to deliver to ' + room_name)
            rospy.loginfo('i am going to deliver to ' + room_name)

            self.move_base_client.wait_for_result()

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = ROOMS[rest[0]]
            goal.target_pose.pose.position.y = ROOMS[rest[1]]
            goal.target_pose.pose.orientation.w = 1  # I don't care about the orientation here
            self.move_base_client.send_goal(goal)
            self.tts_pub.publish('I am going to rest')
            rospy.loginfo('i am going to rest')




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
    rospy.loginfo("what drink and where ")
    speech_control.main_loop()
    rospy.spin()
