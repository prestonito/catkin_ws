#!/usr/bin/env python
# coding=utf-8
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import PoseStamped
from itr_pkg.srv import YOLOLastFrame
from sensor_msgs.msg import Image
import smach
from smach_ros import IntrospectionServer, SimpleActionState
from smach import CBState

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


class RobotParrotSM:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tts_pub = rospy.Publisher('/speech', String, queue_size=50)  # This one uses speech_database
        self.twist = Twist()
        self.place = ''
        self.yolo_srv = rospy.ServiceProxy('/detect_frame', YOLOLastFrame)
        self.img_size = None
        self.image_subs = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_cb)
        # Wait till we got one image
        rate = rospy.Rate(10)
        rospy.loginfo('Waiting for an image...')
        while not self.img_size and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("All is ready!")

    def camera_cb(self, msg):
        self.img_size = msg.width, msg.height
        self.image_subs.unregister()  # This will stop this callback, as we don't need more images!

    def recognize_and_tell(self):
        res = self.yolo_srv()
        for d in res.detections:
            #conf = str(d.confidence*100)  # Use this for a very long decimal and the robot being very robotic
            conf = "{:.2f}".format(d.confidence*100)  # Use this for 2 decimal places
            say = 'I see a ' + d.name + ' with a confidence of ' + str(conf) + ' per cent'

            centre = (d.bbox_x+d.width/2.0, d.bbox_y+d.height/2.0)
            say += ' in the ' + self.get_position_in_image(centre) + ' of the image.'
            self.tts_pub.publish(say)
            rospy.sleep(0.09*len(say))

    def get_position_in_image(self, centre):
        centre_px = 50  # Window of pixels to consider the centre, from the centre of the image
        img_centre = (self.img_size[0]/2.0, self.img_size[1]/2.0)
        horizontal = 'centre'
        if centre[0] < img_centre[0]-centre_px:
            horizontal = 'left'
        elif centre[0] > img_centre[0]+centre_px:
            horizontal = 'right'

        vertical = 'centre'
        if centre[1] < img_centre[1]-centre_px:
            vertical = 'top'
        elif centre[1] > img_centre[1]+centre_px:
            vertical = 'bottom'
        if vertical == horizontal:
            return vertical
        return vertical + ' ' + horizontal

    def main(self):
        sm = smach.StateMachine(outcomes=['succeeded'])
        with sm:
            smach.StateMachine.add('CHECK_SPEECH', GetSpeechState(),
                                   transitions={'cmd_vel': 'MOVE_CMD_VEL',
                                                'move_base': 'SAY_WHERE',
                                                'stop': 'succeeded',
                                                'nothing': 'CHECK_SPEECH'},
                                   remapping={'out_speech': 'speech',
                                              'out_mb_pose': 'move_base_pose'})

            smach.StateMachine.add('MOVE_CMD_VEL', CBState(self.move_cb, cb_args=[self.tts_pub, self.cmd_pub]),
                                   transitions={'succeeded': 'RECOGNIZE_AND_TELL'},
                                   remapping={'in_speech': 'speech'})

            @smach.cb_interface(outcomes=['succeeded'], input_keys=['in_place'])
            def say_mb_cb(ud):
                self.tts_pub.publish('I am going to the ' + ud.in_place)
                return 'succeeded'
            smach.StateMachine.add('SAY_WHERE', CBState(say_mb_cb),
                                   transitions={'succeeded': 'MOVE_BASE'},
                                   remapping={'in_place': 'speech'})

            smach.StateMachine.add('MOVE_BASE', SimpleActionState('/move_base', MoveBaseAction,
                                                                  goal_slots=['target_pose']),
                                   remapping={'target_pose': 'move_base_pose'},
                                   transitions={'succeeded': 'RECOGNIZE_AND_TELL',
                                                'aborted': 'RECOGNIZE_AND_TELL',
                                                'preempted': 'RECOGNIZE_AND_TELL'})

            @smach.cb_interface(outcomes=['succeeded'])
            def recognize_cb(ud):
                self.recognize_and_tell()
                return 'succeeded'
            smach.StateMachine.add('RECOGNIZE_AND_TELL', CBState(recognize_cb),
                                   transitions={'succeeded': 'CHECK_SPEECH'})

        sis = IntrospectionServer('intro_server', sm, 'SM_ROOT')
        sis.start()
        o = sm.execute()
        sis.stop()
        return o

    @smach.cb_interface(outcomes=['succeeded'], input_keys=['in_speech'])
    def move_cb(ud, tts_pub, cmd_vel_pub):
        twist = Twist()
        if ud.in_speech == 'forward':
            # move forward
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            tts_pub.publish('I am moving forward')
        elif ud.in_speech == 'backward':
            # move backward
            twist.linear.x = -0.5
            twist.angular.z = 0.0
            tts_pub.publish('I am moving backward')
        elif ud.in_speech == 'left':
            # move left
            twist.angular.z = 1.0
            twist.angular.x = 0.0
            tts_pub.publish('I am turning left')
        else:  # elif ud.in_speech == 'right':
            # turn right
            twist.angular.z = -1.0
            twist.angular.x = 0.0
            tts_pub.publish('I am turning right')
        cmd_vel_pub.publish(twist)
        return 'succeeded'


class GetSpeechState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cmd_vel', 'move_base', 'nothing', 'stop'], output_keys=['out_speech',
                                                                                                      'out_mb_pose'])
        self.recognized_speech = None
        self.speech_subs = rospy.Subscriber('/speech_recognition/final_result', String, self.speech_received)

    def speech_received(self, msg):
        self.recognized_speech = msg.data

    def execute(self, ud):
        if not self.recognized_speech:
            rospy.sleep(0.5)
            return 'nothing'
        speech = self.recognized_speech
        self.recognized_speech = None
        if 'forward' in speech:
            ud.out_speech = 'forward'
            return 'cmd_vel'
        elif 'backward' in speech:
            ud.out_speech = 'backward'
            return 'cmd_vel'
        elif 'left' in speech:
            ud.out_speech = 'left'
            return 'cmd_vel'
        elif 'right' in speech:
            ud.out_speech = 'right'
            return 'cmd_vel'
        elif 'stop' in speech:
            return 'stop'

        # Then we check for move_base
        words = speech.split(' ')
        for w in words:
            if w in ROOMS:
                coordinates = ROOMS[w]
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.position.x = coordinates[0]
                p.pose.position.y = coordinates[1]
                p.pose.orientation.w = 1  # I don't care about the orientation here
                ud.out_mb_pose = p
                ud.out_speech = w
                return 'move_base'
        rospy.sleep(0.5)
        return 'nothing'  # We didn't understand


if __name__ == "__main__":
    rospy.init_node('robot_parrot', sys.argv)
    parrot = RobotParrotSM()
    rospy.loginfo("I am ready")
    outcome = parrot.main()
    rospy.loginfo('SM ended with outocme: ' + outcome)
