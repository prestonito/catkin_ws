#!/usr/bin/env python

import rospy
import sys
import smach
from std_msgs.msg import String
from smach_ros import IntrospectionServer, SimpleActionState, ServiceState
from move_base_msgs.msg import MoveBaseAction
from itr_tutorials.srv import Str2Coord, Str2CoordRequest
from smach import CBState



#WAITFORREQUEST STATE!!! (we got this from the slide) 
class WaitForRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['request_received','not_request_received','rest_needed'], #the three possible outcomes for the WAIT_FOR_REQUEST state 
                                output_keys = ['out_request_loc','out_resting'],
                                input_keys = ['in_resting']) #userdata allows states to communicate with each other
        
        self.request = []  #i think it makes it empty so far and changes in the execute function
        
        self.request_subs = rospy.Subscriber('/get_me_tea', String, self.request_cb) #subscribes to the topic of /get_me_tea, but where did this topic come from?

    
    def request_cb(self, msg):  #what is msg?
        self.request.append(msg.data)  #adds the msg to a list for a queue

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_FOR_REQUEST with queue: ' + str(self.request))
        
        
        if self.request:
            userdata.out_request_loc = self.request.pop(0) #sets this information outside of just this state
            userdata.out_resting = False   #basically says if we have a request in the queue list, we are no longer resting
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
            smach.StateMachine.add('WAIT_FOR_REQUEST', WaitForRequest(),   #([name of state], [class name])
                                    transitions={'request_received': 'GO_TO_KITCHEN',         #if request_received is returned, transition to GO_TO_KITCHEN 
                                                'not_request_received': 'WAIT_FOR_REQUEST',
                                                'rest_needed':'GO_TO_REST'},
                                    remapping = {'out_request_loc':'request_loc',    #remapping format: ['inside state':'outside state'] 
                                                'in_resting':'resting',
                                                'out_resting':'resting'})  

            smach.StateMachine.add('GO_TO_KITCHEN', GetCoordsAndMoveSM('kitchen'),
                                    transitions = {'succeeded':'GO_TO_REQUEST',
                                                    'aborted':'WAIT_FOR_REQUEST'})

            smach.StateMachine.add('GO_TO_REQUEST', GetCoordsAndMoveSM(),
                                    transitions = {'succeeded':'WAIT_FOR_REQUEST',  #we make it go to request state to see if rest is necessary
                                                    'aborted':'WAIT_FOR_REQUEST'},
                                    remapping = {'in_request':'request_loc'})

            smach.StateMachine.add('GO_TO_REST', GetCoordsAndMoveSM('rest'),
                                    transitions = {'succeeded':'SET_RESTING',
                                                    'aborted':'aborted'})


            #uses a callback state instead of a normal state because its simple and only has one outcome. 
            smach.StateMachine.add('SET_RESTING', CBState(self.set_resting), transitions = {'succeeded':'WAIT_FOR_REQUEST'},
                                    remapping = {'out_resting':'resting'})

        return sm

    @smach.cb_interface(outcomes=['succeeded'], output_keys=['out_resting'])
    def set_resting(userdata):
        userdata.out_resting = True
        return 'succeeded'
    


    def execute_sm(self):

        #this is so you can visually see the state machine
        sm = self.create_sm()
        sis = IntrospectionServer('tea_Server', sm, 'SM_ROOT')   
        sis.start()
        outcome = sm.execute()
        sis.stop()
        return outcome




#this is to simplify the redundancies in our code
class GetCoordsAndMoveSM(smach.StateMachine):
    def __init__(self, fixed_place=None):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted'], input_keys = [] if fixed_place else ['in_request'])   #we know what fixed_place UNLESS from userdata


        with self:
            if fixed_place:
                #this first one is the servicestate for fixed places (rest, kitchen)
                smach.StateMachine.add('GET_COORDS', ServiceState('/str_to_coord', Str2Coord, request = Str2CoordRequest(fixed_place),  #??how does it know what the fixed place coords are???
                                                                        response_slots = ['coordinates']),  #must be the same thing in Str2Coord msg. the Str2Coord msg definition file has two parts: request and response. 'coordinates' is the response part
                                                                        transitions = {'succeeded':'MOVE_TO_PLACE',
                                                                                        'preempted':'aborted'}, 
                                                                        remapping = {'coordinates':'place_coords'})
            else:
                #this second one is the servicestate for places from userdata
                smach.StateMachine.add('GET_COORDS', ServiceState('/str_to_coord', Str2Coord, request_slots = ['place'],  #the Str2Coord service def file has two parts, and 'place' is the request part
                                                                        response_slots = ['coordinates']),  #must be the same thing in Str2Coord msg. the Str2Coord msg definition file has two parts: request and response. 'coordinates' is the response part
                                                                        transitions = {'succeeded':'MOVE_TO_PLACE',
                                                                                        'preempted':'aborted'}, 
                                                                        remapping = {'coordinates':'place_coords',
                                                                                    'place':'in_request'})
            smach.StateMachine.add('MOVE_TO_PLACE', SimpleActionState('/move_base', MoveBaseAction,
                                                                    goal_slots = ['target_pose']),
                                                                    transitions={'succeeded':'succeeded',
                                                                                    'preempted':'aborted',
                                                                                    'aborted':'aborted'},
                                                                    remapping = {'target_pose':'place_coords'})


    





if __name__=='__main__':
    rospy.init_node('go_for_tea_sm', sys.argv) #name it the same as python file name
    instance = GoForTeaSMNode()
    instance.execute_sm()
    rospy.spin