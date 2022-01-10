#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SpeechControl:
    def __init__(self):
        self.speech_subs = rospy.Subscriber('/speech_recognition/final_result', String, self.speech_received) #subscribes to topic that has info about what we're saying
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #creates a publisher that will publish messages on the cmd_vel topic to move robot
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=1) #creates a publisher that will publish what it's doing 
        self.twist = Twist()

    def speech_received(self, msg): #callback that will receive message
        self.recognized_speech = msg.data 
        rospy.loginfo("I heard: " + self.recognized_speech)
        rospy.loginfo(self.recognized_speech)
        if 'forward' in self.recognized_speech:
            # move forward
            self.twist.linear.x = 1.0
            self.tts_pub.publish('I am moving forward')
        elif 'backward' in self.recognized_speech:
            # move backward
            self.twist.linear.x = -1.0
            self.tts_pub.publish('I am moving backward')
        elif 'left' in self.recognized_speech:
            # move left
            self.twist.angular.z = 1.0
            self.tts_pub.publish('I am turning left')
        elif 'right' in self.recognized_speech:
            # turn right
            self.twist.angular.z = -1.0
            self.tts_pub.publish('I am turning right')
        else:
            self.tts_pub.publish('I am stopping')
            self.twist = Twist()

    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.twist)   #publishes the full Twist message to the publisher initialized in the init
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('speech_control', sys.argv)
    speech_control = SpeechControl()
    rospy.loginfo("I am ready")
    speech_control.main_loop()
    rospy.spin()
