#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
import actionlib
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time


class SpeechControl:
    def __init__(self):
        self.speech_subs = rospy.Subscriber('/speech_recognition/final_result', String, self.speech_received) #subscribes to topic that has info about what we're saying
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #creates a publisher that will publish messages on the cmd_vel topic to move robot
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=1) #creates a publisher that will publish what it's doing 
        self.twist = Twist() #what does this do?
        self.pose = Pose()


        #this part below is to initialize a actionlib client
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

    def speech_received(self, msg): #callback that will receive message
        self.recognized_speech = msg.data   #what does this do?
        rospy.loginfo("I heard: " + self.recognized_speech)
        rospy.loginfo(self.recognized_speech)
        goal = MoveBaseGoal()
        if 'office' in self.recognized_speech:
            location = 'office'
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = 2
            goal.target_pose.pose.position.y = 8.5
            goal.target_pose.pose.orientation.w = 1
            
        elif 'bedroom' in self.recognized_speech:
            location = 'bedroom'
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = 6.1
            goal.target_pose.pose.position.y = 8.5
            goal.target_pose.pose.orientation.w = 1
            #self.tts_pub.publish('I am going to bedroom')
        elif 'living room' in self.recognized_speech:
            location = 'living room'
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = 10.7
            goal.target_pose.pose.position.y = 8.4
            goal.target_pose.pose.orientation.w = 1
            #self.tts_pub.publish('I am going to living room')
        elif 'closet' in self.recognized_speech:
            location = 'closet'
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = 2
            goal.target_pose.pose.position.y = 3.25
            goal.target_pose.pose.orientation.w = 1
            #self.tts_pub.publish('I am going to closet')
        elif 'bathroom' in self.recognized_speech:
            location = 'bathroom'
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = 6.1
            goal.target_pose.pose.position.y = 3.5
            goal.target_pose.pose.orientation.w = 1
            #self.tts_pub.publish('I am going to bathroom')
        elif 'kitchen' in self.recognized_speech:
            location = 'kitchen'
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = 10.7
            goal.target_pose.pose.position.y = 2.7
            goal.target_pose.pose.orientation.w = 1
            #self.tts_pub.publish('I am going to kitchen')
        else:
            self.tts_pub.publish('I am stopping')
            self.twist = Twist()
        
        self.move_base_client.send_goal(goal)
        self.tts_pub.publish('I am going to ' + location)
        #time.sleep(3)
        #if self.twist.linear.x == 0:
        #    if self.twist.linear.y == 0:
        #        if self.twist.angular.z == 0:
        #            self.tts_pub.publish('I am in ' + location)

        
        if 0.9 < self.pose.orientation.w < 1.1:
            if (goal.target_pose.pose.position.x + 0.3) < self.pose.position.x < (goal.target_pose.pose.position.x + 0.3):
                if (goal.target_pose.pose.position.y + 0.3) < self.pose.position.y < (goal.target_pose.pose.position.y + 0.3):
                    self.tts_pub.publish('yipee I am in ' + location)



        #return location
        

    #def main_loop(self):
     #   rate = rospy.Rate(10)
      #  while not rospy.is_shutdown():
       #     self.move_base_client.send_goal(goal)   #publishes the full Twist message to the publisher initialized in the init
        #    self.tts_pub.publish('I am in' + location)
         #   rate.sleep()

if __name__ == "__main__":
    rospy.init_node('speech_control', sys.argv)
    speech_control = SpeechControl()
    #self.tts_pub.publish('I am in' + SpeechControl.speech_received())
    rospy.loginfo("I am ready")
    rate = rospy.Rate(10)
    rate.sleep()
    rospy.spin()
