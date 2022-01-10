#!/usr/bin/env python
import rospy
import sys
import smach
from smach_ros import IntrospectionServer, SimpleActionState, ServiceState
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction
from itr_tutorials.srv import Str2Coord, Str2CoordRequest
from smach import CBState

class WaitForRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['request_received', 'not_request_received', 'rest_needed'],
                             output_keys=['out_request_loc', 'out_resting'],
                             input_keys=['in_resting'])
        self.request = []
        self.request_subs = rospy.Subscriber('/get_me_tea', String, self.request_cb)

    def request_cb(self, msg):
        self.request.append(msg.data)

    def execute(self, userdata):
        rospy.loginfo('Executing state WAITFORREQUEST with queue: ' + str(self.request))
        if self.request:
            userdata.out_request_loc = self.request.pop(0)
            userdata.out_resting = False
            return 'request_received'
        else:
            if userdata.in_resting:
                rospy.sleep(0.5)
                return 'not_request_received'
            else:
                return 'rest_needed'


class GoForTeaSMNode:
    def __init__(self):
        pass


    def create_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        sm.userdata.resting = True
        with sm:
            smach.StateMachine.add('WAIT_FOR_REQUEST', WaitForRequest(),
                                   transitions={'request_received': 'GO_TO_KITCHEN',
                                                'not_request_received': 'WAIT_FOR_REQUEST',
                                                'rest_needed': 'GO_TO_REST'},
                                   remapping={'out_request_loc': 'request_loc',
                                              'in_resting': 'resting',
                                              'out_resting': 'resting'})

            smach.StateMachine.add('GO_TO_KITCHEN', GetCoordsAndMoveSM('kitchen'),
                                   transitions={'succeeded': 'GO_TO_REQUEST', 'aborted': 'WAIT_FOR_REQUEST'})

            smach.StateMachine.add('GO_TO_REQUEST', GetCoordsAndMoveSM(),
                                   transitions={'succeeded': 'WAIT_FOR_REQUEST', 'aborted': 'WAIT_FOR_REQUEST'},
                                   remapping={'in_request': 'request_loc'})

            smach.StateMachine.add('GO_TO_REST', GetCoordsAndMoveSM('rest'),
                                   transitions={'succeeded': 'SET_RESTING', 'aborted': 'WAIT_FOR_REQUEST'})

            smach.StateMachine.add('SET_RESTING', CBState(self.set_resting), transitions={'succeeded': 'WAIT_FOR_REQUEST'},
                                   remapping={'out_resting': 'resting'})
        return sm

    @smach.cb_interface(outcomes=['succeeded'], output_keys=['out_resting'])
    def set_resting(userdata):
        userdata.out_resting = True
        return 'succeeded'


    def execute_sm(self):
        sm = self.create_sm()
        sis = IntrospectionServer('tea_Server', sm, 'SM_ROOT')
        sis.start()
        outcome = sm.execute()
        sis.stop()
        return outcome


class GetCoordsAndMoveSM(smach.StateMachine):
    def __init__(self, fixed_place=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[] if fixed_place else ['in_request'])

        with self:
            if fixed_place:
                smach.StateMachine.add('GET_COORDS', ServiceState('/str_to_coord', Str2Coord,
                                                                  request=Str2CoordRequest(fixed_place),
                                                                  response_slots=['coordinates']),
                                       transitions={'succeeded': 'MOVE_TO_PLACE', 'preempted': 'aborted'},
                                       remapping={'coordinates': 'place_coords'})
            else:
                smach.StateMachine.add('GET_COORDS', ServiceState('/str_to_coord', Str2Coord,
                                                                  request_slots=['place'],
                                                                  response_slots=['coordinates']),
                                       transitions={'succeeded': 'MOVE_TO_PLACE', 'preempted': 'aborted'},
                                       remapping={'coordinates': 'place_coords',
                                                  'place': 'in_request'})

            smach.StateMachine.add("MOVE_TO_PLACE", SimpleActionState('/move_base', MoveBaseAction,
                                                                      goal_slots=['target_pose']),
                                   transitions={'succeeded': 'succeeded', 'aborted': 'aborted',
                                                'preempted': 'aborted'},
                                   remapping={'target_pose': 'place_coords'})

if __name__ == "__main__":
    rospy.init_node("go_for_tea_sm", sys.argv)
    get_Tea = GoForTeaSMNode()
    get_Tea.execute_sm()
    rospy.spin()
