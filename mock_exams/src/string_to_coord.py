#!/usr/bin/env python
import rospy 
import sys
from itr_tutorials.srv import Str2Coord, Str2CoordResponse

class StrToCoordNode:
    def __init__(self):

        #creates the service. names it 'str_to_coord' and uses msg type that we created. also has a callback function
        self.coord_service = rospy.Service('/str_to_coord', Str2Coord, self.service_cb)

        #dictionary
        self.coordinates = {'kitchen': [10.7, 2.7],
                            'living room': [10.7, 8.5],
                            'bedroom': [6, 8.5],
                            'office': [2, 8.5],
                            'closet': [1.9, 3.25],
                            'bathroom': [6.2, 3.5],
                            'rest': [10.7, 5.13]}

    def service_cb(self, req):  #creates callback function. it will take a request
        place = req.place  #i think it might the request from the first part of the service def file
        response = Str2CoordResponse()  #its the resposne from the serice definition
        words = place.split(' ')
        for room in words:
            if room in self.coordinates:

                #the response we defined has a message of type geometry_msgs/PoseStamped which is made up of frame_id, time stamp, position, and oreintataion
                response.coordinates.header.frame_id = 'map'
                response.coordinates.header.stamp = rospy.Time.now()
                response.coordinates.pose.position.x = self.coordinates[room][0]
                response.coordinates.pose.position.y = self.coordinates[room][1]
                response.coordinates.pose.orientation.w = 1
                #rospy.loginfo(self.coordinates[room][0] + ' ' + self.coordinates[room][1] + ' ' + response.coordinates.pose.orientation)
            else:
                continue
        rospy.logerr("where tf?")

        return response


if __name__=="__main__":
    rospy.init_node('string_to_coord_srv', sys.argv)  #initialize node, but where did that name come from? 
    instance = StrToCoordNode()
    rospy.loginfo('this ish is running')
    rospy.spin()  #good to include on everything 