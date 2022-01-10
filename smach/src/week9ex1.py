#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool, Header
from smach import StateMachine
from move_base_msgs.msg import MoveBaseGoal

class VacuumCleanerSMNode:
    def __init__(self):
        self.

    def create_sm(self):
        sm = StateMachine(outcomes = ["succeeded", "aborted"])
        with sm:
            smach.StateMachine.add("MOVING", CheckandMoveSM(),
                                    transitions={"obstacle":"TURNING",
                                                "dirt":"SPIRALING",
                                                "low_battery":"GO_TO_CHARGER"})
            
            smach.StateMachine.add("TURNING", CheckandTurnSM(),
                                    transitions={"obstacle":"TURNING",
                                                 "no_obstacle":"MOVING"})
            
            smach.StateMachine.add("SPIRALING", SpirallingState(),
                                    transitions={"no_dirt":"MOVING"})
            
            charger_goal = MoveBaseGoal()
            charger_goal.target_pose = PoseStamped(Header(frame_id='map'), Pose(Point(7.8, 4.2, 0),
                                                                            Quaternion(0, 0, 1, 0)))
            smach.StateMachine.add("GO_TO_CHARGER", SimpleActionState("/move_base", MoveBaseAction,
                                                                        goal = charger_goal),
                                                                        transitions={'succeeded':'CHARGING', 'aborted':'MOVING', 'preempted':'MOVING'})
            
            smach.StateMachine.add("CHARGING", ChargingState(), transitions={'charged':'MOVING',
                                                                             'not_charged':'CHARGING'})



class CheckandMoveSM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome2"])

class CheckandTurnSM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome2"])

class SpirallingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_dirt'])
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.dirt_found = False
        self.dirt_subs = rospy.Subscriber('/dirt', Bool, self.dirt_cb)



if __name__="__main__";
    rospy.init_node