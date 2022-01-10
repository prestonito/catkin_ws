#!/usr/bin/env python
import rospy
import sys
import smach
from std_msgs.msg import String
from smach_ros import IntrospectionServer, SimpleActionState, ServiceState
from move_base_msgs.msg import MoveBaseAction
from itr_tutorials.srv import Str2Coord, Str2CoordRequest



#WAITFORREQUEST STATE!!! (we got this from the slide) 
class WaitForRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['request_received','not_request_received'], #the two possible outcomes for the WAIT_FOR_REQUEST state is either request receieved or not
                                output_keys = ['in_request_loc']) #userdata allows states to communicate with each other
        
        self.request = []  #i think it makes it empty so far and changes in the execute function
        
        self.request_subs = rospy.Subscriber('/get_me_tea', String, self.request_cb) #subscribes to the topic of /get_me_tea, but where did this topic come from?

    
    def request_cb(self, msg):  #what is msg?
        self.request.append(msg.data)  #adds the msg to a list for a queue

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_FOR_REQUEST with queue: ' + str(self.request))
        if self.request:
            userdata.in_request_loc = self.request.pop(0) #sets this information outside of just this state
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
            smach.StateMachine.add('WAIT_FOR_REQUEST', WaitForRequest(),   #([name of state], [class name])
                                    transitions={'request_received': 'GET_KITCHEN_COORDS',         #if request_received is returned, transition to GO_TO_KITCHEN 
                                                'not_request_received': 'WAIT_FOR_REQUEST'},
                                    remapping = {'in_request_loc':'request_loc'})      #remapping = {'name within this state':'name outside of the state'}

            smach.StateMachine.add('GET_KITCHEN_COORDS', ServiceState('/str_to_coord', Str2Coord, request = Str2CoordRequest('kitchen'), #FOR SERVICE STATES: ([name of state], [ServiceState(service type, message type, request, response_slots, transitions, remapping)]) <---this is from the powerpoint
                                                                        #response slots is the ServiceState way of assinging userdata
                                                                        response_slots = ['coordinates']),  #must be the same thing in Str2Coord service. the Str2Coord definition file has two parts: request and response. 'coordinates' is the response part
                                                                        transitions = {'succeeded':'GO_TO_KITCHEN',
                                                                                        'preempted':'aborted'}, 
                                                                        remapping = {'coordinates':'kitchen_coord'})  #for remapping: {[within the sm], [outside of the state]}
            
            smach.StateMachine.add('GO_TO_KITCHEN', SimpleActionState('/move_base', MoveBaseAction, 
                                                                        goal_slots = ['target_pose']),  #telling it which message it needs to fill. rosmsg show MoveBaseGoal shows that it's target_pose. similar to resposne slots in service state above
                                                                        transitions = {'succeeded':'GET_REQUEST_COORDS',
                                                                                        'preempted':'WAIT_FOR_REQUEST',
                                                                                        'aborted':'WAIT_FOR_REQUEST'},
                                                                        remapping = {'target_pose':'kitchen_coord'}) #in this state, it MUST be called 'target_pose' because that's what the goal slots is. 


            #this state is another servicestate to get the coords of the request room. main difference is that this one doesn't have a fixed request, so we must use request slots  (information that is available on the powerpoint)
            smach.StateMachine.add('GET_REQUEST_COORDS', ServiceState('/str_to_coord', Str2Coord, request_slots = ['place'],  #the Str2Coord service def file has two parts, and 'place' is the request part
                                                                        response_slots = ['coordinates']),  #must be the same thing in Str2Coord msg. the Str2Coord msg definition file has two parts: request and response. 'coordinates' is the response part
                                                                        transitions = {'succeeded':'GO_TO_REQUEST',
                                                                                        'preempted':'aborted'}, 
                                                                        remapping = {'coordinates':'request_coord',
                                                                                        'place':'request_loc'})  #for remapping: {[within the sm], [outside of the state]}
            
            
            #this state is another simpleactionstate to go to request location. must change the remapping 
            smach.StateMachine.add('GO_TO_REQUEST', SimpleActionState('/move_base', MoveBaseAction, 
                                                                        goal_slots = ['target_pose']),  #telling it which message it needs to fill. rosmsg show MoveBaseGoal shows that it's target_pose. similar to resposne slots in service state above
                                                                        transitions = {'succeeded':'GET_REST_COORDS',
                                                                                        'preempted':'aborted',
                                                                                        'aborted':'aborted'},
                                                                        remapping = {'target_pose':'request_coord'}) #in this state, it MUST be called 'target_pose' because that's what the goal slots is. 

            smach.StateMachine.add('GET_REST_COORDS', ServiceState('/str_to_coord', Str2Coord, request = Str2CoordRequest('rest'),  #the Str2Coord service def file has two parts, and 'place' is the request part
                                                                        response_slots = ['coordinates']),  #must be the same thing in Str2Coord msg. the Str2Coord msg definition file has two parts: request and response. 'coordinates' is the response part
                                                                        transitions = {'succeeded':'GO_TO_REST',
                                                                                        'preempted':'aborted'}, 
                                                                        remapping = {'coordinates':'rest_coord'})  #for remapping: {[within the sm], [outside of the state]}
            
            
            smach.StateMachine.add('GO_TO_REST', SimpleActionState('/move_base', MoveBaseAction,
                                                                    goal_slots = ['target_pose']),
                                                                    transitions={'succeeded':'WAIT_FOR_REQUEST',
                                                                                    'preempted':'aborted',
                                                                                    'aborted':'aborted'},
                                                                    remapping = {'target_pose':'rest_coord'})

        return sm

    def execute_sm(self):

        #this is so you can visually see the state machine
        sm = self.create_sm()
        sis = IntrospectionServer('tea_Server', sm, 'SM_ROOT')   
        sis.start()
        outcome = sm.execute()
        sis.stop()
        return outcome





if __name__=='__main__':
    rospy.init_node('go_for_tea_sm', sys.argv) #name it the same as python file name
    instance = GoForTeaSMNode()
    instance.execute_sm()
    rospy.spin