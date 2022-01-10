#!/usr/bin/env python
import rospy
import sys
import smach
from smach_ros import IntrospectionServer, SimpleActionState, ServiceState
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction
from itr_tutorials.srv import Str2Coord, Str2CoordRequest


class WaitForRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['request_received', 'not_request_received'], output_keys=['in_request_loc'])
        self.request = []
        self.request_subs = rospy.Subscriber('/get_me_tea', String, self.request_cb)

    def request_cb(self, msg):
        self.request.append(msg.data)

    def execute(self, userdata):
        rospy.loginfo('Executing state WAITFORREQUEST with queue: ' + str(self.request))
        if self.request:
            userdata.in_request_loc = self.request.pop(0)
            return 'request_received'
        else:
            rospy.sleep(0.5)
            return 'not_request_received'


class GoForTeaSMNode:
    def __init__(self):
        pass


    def create_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        with sm:
            smach.StateMachine.add('WAIT_FOR_REQUEST', WaitForRequest(),
                                   transitions={'request_received': 'GET_KITCHEN_COORDS',
                                                'not_request_received': 'WAIT_FOR_REQUEST'},
                                   remapping={'in_request_loc': 'request_loc'})

            smach.StateMachine.add('GET_KITCHEN_COORDS', ServiceState('/str_to_coord', Str2Coord,
                                                                      request=Str2CoordRequest('kitchen'),
                                                                      response_slots=['coordinates']),
                                   transitions={'succeeded': 'GO_TO_KITCHEN', 'preempted': 'aborted'},
                                   remapping={'coordinates': 'kitchen_coord'})

            smach.StateMachine.add("GO_TO_KITCHEN", SimpleActionState('/move_base', MoveBaseAction,
                                                                      goal_slots=['target_pose']),
                                   transitions={'succeeded': 'GET_REQUEST_COORDS', 'aborted': 'WAIT_FOR_REQUEST', 'preempted': 'WAIT_FOR_REQUEST'},
                                   remapping={'target_pose': 'kitchen_coord'})

            smach.StateMachine.add('GET_REQUEST_COORDS', ServiceState('/str_to_coord', Str2Coord,
                                                                      request_slots=['place'],
                                                                      response_slots=['coordinates']),
                                   transitions={'succeeded': 'GO_TO_REQUEST', 'preempted': 'aborted'},
                                   remapping={'coordinates': 'request_coord',
                                              'place': 'request_loc'})

            smach.StateMachine.add("GO_TO_REQUEST", SimpleActionState('/move_base', MoveBaseAction,
                                                                      goal_slots=['target_pose']),
                                   transitions={'succeeded': 'GET_REST_COORDS', 'aborted': 'WAIT_FOR_REQUEST', 'preempted': 'WAIT_FOR_REQUEST'},
                                   remapping={'target_pose': 'request_coord'})

            smach.StateMachine.add('GET_REST_COORDS', ServiceState('/str_to_coord', Str2Coord,
                                                                      request=Str2CoordRequest('rest'),
                                                                      response_slots=['coordinates']),
                                   transitions={'succeeded': 'GO_TO_REST', 'preempted': 'aborted'},
                                   remapping={'coordinates': 'rest_coord'})

            smach.StateMachine.add("GO_TO_REST", SimpleActionState('/move_base', MoveBaseAction,
                                                                      goal_slots=['target_pose']),
                                   transitions={'succeeded': 'WAIT_FOR_REQUEST', 'aborted': 'WAIT_FOR_REQUEST',
                                                'preempted': 'WAIT_FOR_REQUEST'},
                                   remapping={'target_pose': 'rest_coord'})
        return sm

    def execute_sm(self):
        sm = self.create_sm()
        sis = IntrospectionServer('tea_Server', sm, 'SM_ROOT')
        sis.start()
        outcome = sm.execute()
        sis.stop()
        return outcome

if __name__ == "__main__":
    rospy.init_node("go_for_tea_sm", sys.argv)
    get_Tea = GoForTeaSMNode()
    get_Tea.execute_sm()
    rospy.spin()
