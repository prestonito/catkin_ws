#!/usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
#from itr_yolo_pkg import YOLOLastFrame, YOLOLastFrameResponse, YOLODetection
from yolov4 import Detector

from cv_bridge3 import CvBridge
from cv_bridge3 import cv2



class YOLOV4ROSITR:
    def __init__(self):
        self.cv_image = None
        self.cam_subs = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        #self.yolo_srv = rospy.Service("/detect_frame", YOLOLastFrame, self.yolo_service)
        #self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
         #                   weights_path='/opt/darknet/yolov4.weights',
          #                  lib_darknet_path='/opt/darknet/libdarknet.so',
           #                 meta_path='/home/preston/catkin_ws/src/yolo_itr_pkg/config/coco.data')

    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        rospy.loginfo("I have received a new image.")

    def yolo_service(sefl, request):
        #res = YOLOLastFrameResponse()

        if not self.cv_image:
            rospy.logwarn("i have not yet received any image")
            return False

            

        return True
        
    


if __name__ == '__main__':
    rospy.init_node("test_yolo_ros_itr")
    test_yolo_ros = YOLOV4ROSITR()
    rospy.spin()