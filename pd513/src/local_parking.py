#!/usr/bin/env python

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
        self.img_sub = rospy.Subscriber('/none', Image, self.img_callback)
        self.pose_sub = rospy.Subscriber('/pose', Pose, self.pose_callback)
        self.local_sub = rospy.Subscriber('/local_cmd', Empty, self.local_callback)
        self.stop_pub = rospy.Publisher('/stop', Empty, queue_size=10)

        self.is_local = False

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

        self.min_area = 740
        self.direction = 0 

        # self.backward_dist = 410
        self.backward_dist = 411
        self.forward_dist = 356

        # self.front_cam_topic_name = '/front_cam/image_raw'
        self.front_cam_topic_name = '/front_cam/image_raw'
        self.rear_cam_topic_name = '/rear_cam/image_raw'

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
          
            square_yellow, is_square = self.detect_square(input_yellow)

            if is_square:
                center_yellow = np.int0(np.mean(square_yellow, axis=0))
           
                cv2.drawContours(input_warped, [square_yellow], 0, (0,0,255), 3, cv2.LINE_AA)
            
                y1 = square_yellow[1][1]
                y2 = square_yellow[2][1]

                dist = int((y1 + y2) / 2.0)

                if self.direction == 1:
                    if dist > self.forward_dist:
                        print('stop.')
                        self.stop_pub.publish(Empty())
                        self.img_sub.unregister()
                    else:
                        print('distance to stop line: %d' % (480 - dist))
                
                if self.direction == 2:
                    if dist > self.backward_dist:
                        print('stop.')
                        self.stop_pub.publish(Empty())
                        self.img_sub.unregister()
                    else:
                        print('distance to stop line: %d' % (480 - dist))

            # cv2.imshow('yellow', input_yellow)
            # cv2.imshow('local parking space', input_)
            cv2.imshow('local parking space warped', input_warped)
            
            cv2.waitKey(1)
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
        output[imask] = 255

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
