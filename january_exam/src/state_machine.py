
# This code provides snippets you can use to build your solutions, and to save you some time. You can copy snippets from here into your solution in the relevant places


##  Relevant imports
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState



GOALS = {'first': [1.67464196682, 8.84763145447],   # A
         'second': [5.94639539719, 8.83150196075],    # B
         'third': [10.5727844238, 8.31532382965],    # C
         'fourth': [1.80360078812, 3.86330342293],   # D
         'fifth': [5.96251535416, 3.1858215332],  # E
         'sixth': [10.5405454636, 3.10516905785]}  # F



class GetGoalCoords:
	def __init__(self):

		self.list = []

	def add_to_list(self):

		for goal in GOALS:
			x_coord = GOALS[0]
			y_coord = GOALS[1]

		return 'request_received'



## To define a state machine with one outcome to succeed:
sm = smach.StateMachine(outcomes = ['succeeded', 'aborted'])
with sm:
	# Here you add the definitions of your states.
	smach.StateMachine.add('GET_GOAL_COORDS', GetGoalCoords(),
							transitions = {'request_received':'GO_TO_GOAL'})



	#i know this method defeats the purpose of using state machines but i think it might work if i had time to finish implementing so i wanted to 
	#show that i think this would work
	smach.StateMachine.add('GO_TO_1', SimpleActionState,
							goal_slots = ['target_pose']),
							transitions = {'succeeded':'first'},
							remapping = {'target_pose':'1'}
	smach.StateMachine.add('GO_TO_2', SimpleActionState,
							goal_slots = ['target_pose']),
							transitions = {'succeeded':'first'},
							remapping = {'target_pose':'2'}
	smach.StateMachine.add('GO_TO_3', SimpleActionState,
							goal_slots = ['target_pose']),
							transitions = {'succeeded':'first'},
							remapping = {'target_pose':'3'}
				
	smach.StateMachine.add('GO_TO_4', SimpleActionState,
							goal_slots = ['target_pose']),
							transitions = {'succeeded':'first'},
							remapping = {'target_pose':'4'}
	smach.StateMachine.add('GO_TO_5', SimpleActionState,
							goal_slots = ['target_pose']),
							transitions = {'succeeded':'first'},
							remapping = {'target_pose':'5'}
	smach.StateMachine.add('GO_TO_6', SimpleActionState,
							goal_slots = ['target_pose']),
							transitions = {'succeeded':'first'},
							remapping = {'target_pose':'6'}
## To add an introspection server to be able to visualize the state machine:
sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

## To execute a state machine:
sm.execute()
