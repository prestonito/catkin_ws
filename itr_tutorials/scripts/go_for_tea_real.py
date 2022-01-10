#!/usr/bin/env python
import rospy
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from itr_tutorials.srv import Str2Coord
from itr_tutorials.msg import GetTeaAction, GetTeaFeedback, GetTeaResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class GoForTeaNode:
    def __init__(self):
        rospy.loginfo("Preparing everything")
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) #creates actionlib client
        self.move_base_client.wait_for_server()

        self.coord_srv = rospy.ServiceProxy('/str_to_coord', Str2Coord) #basically a client to the service we created (string to coord)
        self.coord_srv.wait_for_service()

        self.kitchen_coords = self.coord_srv('kitchen').coordinates #gets the 'room' coordinates from the service we created
        self.rest_coords = self.coord_srv('rest').coordinates

        self.speech_pub = rospy.Publisher('/speech', String, queue_size=2)  # Using speech_database TTS

        self.go_tea_action = actionlib.SimpleActionServer('/get_me_tea', GetTeaAction, self.go_get_tea, auto_start=False) #creates actionlib server with our own generated messages
        self.go_tea_action.start()

        rospy.loginfo("Ready to get tea")

    def go_get_tea(self, goal):
        place = goal.destination #the request part of the service def file = goal part of the action def file
        feedback = GetTeaFeedback()

        place_coords = self.coord_srv(place).coordinates #this variable is called later to get the coords of the target rooms
        
        #this part below is for when the goal is a place that it doesn't know
        if not place_coords.header.frame_id:
            rospy.logerr("I can't go to " + place)
            # PUBLISH FEEDBACK
            feedback.what = "Don't know where " + place + " is."
            self.speech_pub.publish('I am sorry, I ' + feedback.what)
            feedback.next_pose = PoseStamped()
            self.go_tea_action.publish_feedback(feedback)
            self.go_tea_action.set_aborted(GetTeaResult(False), "Don't know where " + place + " is.")
            return

        
        

        
        
        
        ###GOING TO KITCHEN SECTION###
        
        kitchen_goal = MoveBaseGoal() #makes "kitchen_goal" the MoveBaseGoal. using rosmsg show MoveBaseGoal,
                                        #we can see that its a geometry_msgs/PoseStamped and a target_pose
        kitchen_goal.target_pose = self.kitchen_coords #assigning the target_pose (from MoveBaseGoal) to kitchen coords from the service
        self.move_base_client.send_goal(kitchen_goal)

        # PUBLISH FEEDBACK
        feedback.what = "Going to the kitchen"  #we made our feedback in the action def file have two parts: string what and goemetry_msgs next pose
        feedback.next_pose = self.kitchen_coords
        self.speech_pub.publish('I am ' + feedback.what)
        self.go_tea_action.publish_feedback(feedback)

        # WAIT FOR END OF MOVEMENT
        self.move_base_client.wait_for_result()

        status = self.move_base_client.get_state()
       
       #this part below is if there is an error in going to kitchen
        if status != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn("Could not go to the kitchen")
            # PUBLISH FEEDBACK
            feedback.what = "Failed to go to the kitchen"
            self.speech_pub.publish('I am sorry, I ' + feedback.what)
            feedback.next_pose = self.kitchen_coords
            self.go_tea_action.publish_feedback(feedback)
            self.go_tea_action.set_aborted(GetTeaResult(False), "Failed to reach the kitchen")
            return

       
       
       
       
        ###GOING TO TARGET ROOM SECTION###
       
        goal = MoveBaseGoal()
        goal.target_pose = place_coords   #sets the target pose of the target room using the server
        self.move_base_client.send_goal(goal)
        # PUBLISH FEEDBACK
        feedback.what = "Going to " + place
        self.speech_pub.publish('I am ' + feedback.what)
        feedback.next_pose = place_coords
        self.go_tea_action.publish_feedback(feedback)

        # WAIT FOR END OF MOVEMENT
        self.move_base_client.wait_for_result()

        status = self.move_base_client.get_state()
       
       #this part below is if it couldn't reach the target rooms
        if status != actionlib.GoalStatus.SUCCEEDED:
            rospy.logerr("Couldn't reach " + place)
            # PUBLISH FEEDBACK
            feedback.what = "Couldn't reach " + place
            self.speech_pub.publish('I am sorry, I' + feedback.what)
            feedback.next_pose = PoseStamped()
            self.go_tea_action.publish_feedback(feedback)
            self.go_tea_action.set_aborted(GetTeaResult(False), "Don't know where " + place + " is.")
            return




        ##GOING TO REST SECTION##

        rest_goal = MoveBaseGoal()
        rest_goal.target_pose = self.rest_coords
        self.move_base_client.send_goal(rest_goal)
        # PUBLISH FEEDBACK
        feedback.what = "Going to rest"
        self.speech_pub.publish('I am now ' + feedback.what)
        feedback.next_pose = self.rest_coords
        self.go_tea_action.publish_feedback(feedback)

        # WAIT FOR END OF MOVEMENT
        self.move_base_client.wait_for_result()

        status = self.move_base_client.get_state()
        if status != actionlib.GoalStatus.SUCCEEDED:
            rospy.logerr("I couldn't go to rest")
            # PUBLISH FEEDBACK
            feedback.what = "Couldn't reach the rest position"
            self.speech_pub.publish('I am sorry, I ' + feedback.what)
            feedback.next_pose = PoseStamped()
            self.go_tea_action.publish_feedback(feedback)
            self.go_tea_action.set_aborted(GetTeaResult(False), "Don't know where " + place + " is.")
            return
        self.speech_pub.publish('I completed the order')
        self.go_tea_action.set_succeeded(GetTeaResult(True), "Completed the order")


if __name__ == "__main__":
    rospy.init_node("go_for_tea_node", sys.argv)
    gft = GoForTeaNode()
    rospy.spin()
