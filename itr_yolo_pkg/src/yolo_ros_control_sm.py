#!/usr/bin/env python
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>
import rospy
import sys
import smach
from smach_ros import ServiceState, IntrospectionServer
from itr_pkg.srv import YOLOLastFrame  # Change this to your package name
from geometry_msgs.msg import Twist


# I am implementing this differently. Now I just create a class that inherits from StateMachine, and I will call execute
# on that from the main
class YOLOControlNodeSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted'])
        with self:
            # Request is empty, so we do not need to do anything for the request
            smach.StateMachine.add('FIND_OBJECT', ServiceState('/detect_frame', YOLOLastFrame,
                                                                    response_slots=['detections']),
                                   transitions={'succeeded': 'MOVE_ROBOT',
                                                'preempted': 'FIND_OBJECT',
                                                'aborted': 'aborted'})

            # If I do not do any remapping, the same variable can be used in the userdata. however, if we don't declare
            # it as both input and output key, we won't be able to write on that value.
            # If we see a bottle, we will end the SM (and the program)
            smach.StateMachine.add('MOVE_ROBOT', MoveRobotState(), transitions={'succeeded': 'FIND_OBJECT',
                                                                                'stop': 'succeeded'})


class MoveRobotState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'stop'], input_keys=['detections'])
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def execute(self, ud):  # ud is the userdata
        twist = Twist()
        for detection in ud.detections:
            rospy.loginfo("I saw a " + detection.name + " with a confidence of " + str(detection.confidence))
            if 'cup' in detection.name or 'glass' in detection.name:
                twist.linear.x = 5
                twist.angular.z = 0
            elif 'cell phone' in detection.name:
                twist.linear.x = 0
                twist.angular.z = 2
            elif 'bottle' in detection.name:
                return 'stop'
            self.robot_move_pub.publish(twist)
        return 'succeeded'


if __name__ == "__main__":
    rospy.init_node("yolo_control_sm", sys.argv)
    sm = YOLOControlNodeSM()
    sis = IntrospectionServer('iserver', sm, 'SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
    rospy.loginfo('I have completed execution with outcome: ' + outcome)
