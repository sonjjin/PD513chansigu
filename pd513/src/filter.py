#!/usr/bin/env python

import cv2 as cv
import cv2
import math
import numpy as np

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32, Empty
from cv_bridge import CvBridge

class Parking:
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.img_sub = rospy.Subscriber('/front_cam/image_raw', Image, self.img_callback)
        self.pose_sub = rospy.Subscriber('/pose', Pose, self.pose_callback)
        self.local_sub = rospy.Subscriber('/local_cmd', Empty, self.local_callback)
        self.stop_pub = rospy.Publisher('/stop', Empty, queue_size=10)

        self.is_local = True
	self.is_first = True
        self.forward_src = np.float32([
            (133, 89),
            (506, 89),
            (-100, 480),
            (740, 480)
        ])

        self.forward_dst = np.float32([
            (100, 0),
            (540, 0),
            (100, 480),
            (540, 480)
        ])
        
        self.backward_src = np.float32([
            (140, 89),
            (460, 89),
            (-106, 480),
            (740, 480)
        ])

        self.backward_dst = np.float32([
            (100, 0),
            (540, 0),
            (100, 480),
            (540, 480)
        ])

        self.direction = 1 

        self.front_cam_topic_name = '/front_cam/image_raw'
        self.rear_cam_topic_name = '/rear_cam/image_raw'

	self.feature_params = dict(maxCorners = 100,
		qualityLevel = 0.3,
                minDistance = 7,
                blockSize = 7)

	self.lk_params = dict(winSize = (15,15),
		maxLevel = 2,
		criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
        

    def pose_callback(self, data):
        self.direction = data.orientation.y
        if self.direction == 1:
            print('forward')
            self.img_sub = rospy.Subscriber(self.front_cam_topic_name, Image, self.img_callback)
        elif self.direction == 2:
            print('backward')
            self.img_sub = rospy.Subscriber(self.rear_cam_topic_name, Image, self.img_callback)

        self.pose_sub.unregister()

    def img_callback(self, data):
        if self.is_local:    
            input = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
            input = cv2.cvtColor(input, cv2.COLOR_BGR2RGB)
            input_ = input.copy() 
           
            if self.direction == 1:
                input_ = cv2.line(input_, (self.forward_src[0][0], self.forward_src[0][1]), (self.forward_src[1][0], self.forward_src[1][1]), (255, 0, 0), 3, cv2.LINE_AA)
                input_ = cv2.line(input_, (self.forward_src[1][0], self.forward_src[1][1]), (self.forward_src[3][0], self.forward_src[3][1]), (255, 0, 0), 3, cv2.LINE_AA)
                input_ = cv2.line(input_, (self.forward_src[3][0], self.forward_src[3][1]), (self.forward_src[2][0], self.forward_src[2][1]), (255, 0, 0), 3, cv2.LINE_AA)
                input_ = cv2.line(input_, (self.forward_src[2][0], self.forward_src[2][1]), (self.forward_src[0][0], self.forward_src[0][1]), (255, 0, 0), 3, cv2.LINE_AA)        
                
                input_warped, _, _ = self.warp(input, self.forward_src, self.forward_dst)
            elif self.direction == 2:
                input_ = cv2.line(input_, (self.backward_src[0][0], self.backward_src[0][1]), (self.backward_src[1][0], self.backward_src[1][1]), (255, 0, 0), 3, cv2.LINE_AA)
                input_ = cv2.line(input_, (self.backward_src[1][0], self.backward_src[1][1]), (self.backward_src[3][0], self.backward_src[3][1]), (255, 0, 0), 3, cv2.LINE_AA)
                input_ = cv2.line(input_, (self.backward_src[3][0], self.backward_src[3][1]), (self.backward_src[2][0], self.backward_src[2][1]), (255, 0, 0), 3, cv2.LINE_AA)
                input_ = cv2.line(input_, (self.backward_src[2][0], self.backward_src[2][1]), (self.backward_src[0][0], self.backward_src[0][1]), (255, 0, 0), 3, cv2.LINE_AA)        
                
                input_warped, _, _ = self.warp(input, self.backward_src, self.backward_dst)
                

            input_yellow = self.hsv(input_warped, color='yellow')  
            input_green = self.hsv(input_warped, color='green')

            input_total = cv2.add(input_yellow, input_green)
            input_total = input_total[:340,:]

	    
	    if self.is_first:
	        self.is_fist = False
		self.old_input_total = input_total
	    try:
		    p0 = cv2.goodFeaturesToTrack(self.old_input_total, mask = None, **self.feature_params)
		    
		    for ii in range(p0.shape[0]):			
			out_frame = cv2.circle(input_warped, (p0[ii][0][0],p0[ii][0][1]), 10, (0,0,255), 20)
	    except:
		print('no point\n')

            cv2.imshow('total', input_total)
            cv2.imshow('yellow', input_yellow)
            cv2.imshow('green', input_green)
            cv2.imshow('warped', input_warped)
            # cv2.imshow('local parking space', input_)
            # cv2.imshow('local parking space warped', input_warped)
            
            cv2.waitKey(1)

	    self.old_input_total = input_total
        else:
            print('wait for calling.')

    def local_callback(self, data):
        self.is_local = True
        print('local parking node called.')
        self.local_sub.unregister()

    def warp(self, img, src, dst):
        H, W = img.shape[:2]
        mat_M = cv2.getPerspectiveTransform(src, dst)
        mat_M_inv = cv2.getPerspectiveTransform(dst, src)
        img_warped = cv2.warpPerspective(img, mat_M, (W, H), flags=cv2.INTER_LINEAR)
       
        img_warped = img_warped[int(dst[0][1]):int(dst[3][1]), int(dst[0][0]):int(dst[3][0])]

        return img_warped, mat_M, mat_M_inv

    def hsv(self, img, color='green'):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        if color == 'green':
            mask = cv2.inRange(hsv, (25, 60, 50), (86, 255, 255))
        elif color == 'red':
            mask = cv2.inRange(hsv, (115, 100, 50), (130, 255, 255))
        elif color == 'blue':
            mask = cv2.inRange(hsv, (10, 150, 50), (30, 255, 255))
        elif color == 'yellow':
            # mask = cv2.inRange(hsv, (80, 40, 145), (150, 255, 255))
            mask = cv2.inRange(hsv, (80, 100, 145), (150, 255, 255))

        imask = mask > 0
        output = np.zeros_like(hsv, np.uint8)
        if color == 'yellow':
            output[imask] = 255
        elif color == 'green':
            output[imask] = 122

        return output[:,:,0]
    
    def detect_square(self, input):
        H, W = input.shape[:2]

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
        clean = cv2.morphologyEx(input, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
        clean = cv2.morphologyEx(input, cv2.MORPH_CLOSE, kernel)

        contours = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        square = None
        square_center = 0
        is_square = False

        for c in contours:
            rot_rect = cv2.minAreaRect(c)
            temp_area = rot_rect[1][0] * rot_rect[1][1]
            temp_square = cv2.boxPoints(rot_rect)
            temp_center = np.int0(np.mean(temp_square, axis=0))

            if temp_area >= self.min_area and temp_center[0] > square_center:
                square = np.int0(temp_square)
                square_center = temp_center[0]
                area = temp_area
                is_square = True

        return square, is_square

if __name__ == '__main__':
    rospy.init_node('local_parking_node')
    r = rospy.Rate(10)
    p = Parking()

    rospy.spin()
