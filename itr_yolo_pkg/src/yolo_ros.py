#!/usr/bin/env python3
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>
# coding=utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge3 import CvBridge  # Note: normally this would be from cv_bridge import CvBridge
from cv_bridge3 import cv2  # this should be import cv2
from itr_yolo_pkg.srv import YOLOLastFrame, YOLOLastFrameResponse
from itr_yolo_pkg.msg import YOLODetection
import numpy as np
from yolov4 import Detector
import random


class YOLOv4ROSITR:
    def __init__(self):
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='/home/preston/catkin_ws/src/itr_yolo_pkg/config/coco.data')

        self.bridge = CvBridge()
        self.cam_subs = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.yolo_srv = rospy.Service('/detect_frame', YOLOLastFrame, self.yolo_service)

        self.pub = rospy.Publisher("/test_image", Image, queue_size=1)
        self.cv_image = None
        self.colors = {}


    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        #rospy.loginfo("received image")

    def yolo_service(self, request):
        res = YOLOLastFrameResponse()
        if self.cv_image is not None:
            cv_copy = self.cv_image.copy()
            cv_height, cv_width, _ = cv_copy.shape
            img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
            for detection in detections:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')
                d = YOLODetection(detection.class_name, detection.class_confidence, detection.left_x, detection.top_y,
                                  detection.width, detection.height)
                # convert bbox to image space
                d.bbox_x = int((d.bbox_x/self.detector.network_width())*cv_width)
                d.bbox_y = int((d.bbox_y/self.detector.network_height())*cv_height)
                d.width = int((d.width/self.detector.network_width())*cv_width)
                d.height = int((d.height/self.detector.network_height())*cv_height)
                res.detections.append(d)

                if d.name in self.colors:
                    color = self.colors[d.name]
                else:
                    color = (random.randint(0,255), random.randint(0, 255), random.randint(0,255))
                    self.colors[d.name] = color
                cv2.rectangle(cv_copy, (d.bbox_x, d.bbox_y), (d.bbox_x+d.width, d.bbox_y+d.height), color, 2)
                cv2.rectangle(cv_copy, (d.bbox_x, d.bbox_y), (d.bbox_x+5+(23*len(d.name)), d.bbox_y+30), color, -1)
                cv2.putText(cv_copy, d.name, (d.bbox_x+2, d.bbox_y+25), cv2.FONT_HERSHEY_PLAIN, 2.5, (0,0,0))
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_copy))

        return res


if __name__ == '__main__':
    rospy.init_node('solution_yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()
