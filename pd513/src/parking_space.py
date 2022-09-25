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
        self.img_sub = rospy.Subscriber('/test_cam/image_raw', Image, self.img_callback)
        self.stop_sub = rospy.Subscriber('/stop', Empty, self.stop_callback)
        self.pose_pub = rospy.Publisher('/pose', Pose, queue_size=10)

        self.src = np.float32([
            (200, 0),
            (1500, 0),
            (0, 1080),
            (1670, 1080)
        ])

        self.dst = np.float32([
            (300, 0),
            (1620, 0),
            (300, 1080),
            (1620, 1080)
        ])

        self.parking_spot = -1
        
        self.xpix_ratio = None
        self.ypix_ratio = None

        self.min_area = 200

        # 1: forward / 2: backward
        self.direction = 2

        self.is_stop = 0

    def stop_callback(self, data):
        self.is_stop = 1
        print('stop callback called.')

    def img_callback(self, data):
        input = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
        input = cv2.cvtColor(input, cv2.COLOR_BGR2RGB)
        input_ = input.copy()
        
        input_ = cv2.line(input_, (self.src[0][0], self.src[0][1]), (self.src[1][0], self.src[1][1]), (255, 0, 0), 3, cv2.LINE_AA)
        input_ = cv2.line(input_, (self.src[1][0], self.src[1][1]), (self.src[3][0], self.src[3][1]), (255, 0, 0), 3, cv2.LINE_AA)
        input_ = cv2.line(input_, (self.src[3][0], self.src[3][1]), (self.src[2][0], self.src[2][1]), (255, 0, 0), 3, cv2.LINE_AA)
        input_ = cv2.line(input_, (self.src[2][0], self.src[2][1]), (self.src[0][0], self.src[0][1]), (255, 0, 0), 3, cv2.LINE_AA)        
        
        input_warped, _, _ = self.warp(input, self.src, self.dst)
        input_warped_ = input_warped.copy()

        input_green = self.hsv(input_warped, color='green')  
        
        row_up, row_down, cols_down, cols_up = self.detect_lines(input_green)

        if self.xpix_ratio == None and self.ypix_ratio == None:
            self.xpix_ratio = 318.5 / abs(cols_up[0] - cols_up[4])
            self.ypix_ratio = 250.0 / abs(row_up - row_down)
        
        cv2.line(input_warped_, (cols_down, row_down), (cols_down, row_down), (0, 255, 255), 10, cv2.LINE_AA)
        cv2.line(input_warped_, (cols_up[0], row_up), (cols_up[0], row_up), (0, 255, 255), 10, cv2.LINE_AA)
        cv2.line(input_warped_, (cols_up[1], row_up), (cols_up[1], row_up), (0, 255, 255), 10, cv2.LINE_AA)
        cv2.line(input_warped_, (cols_up[2], row_up), (cols_up[2], row_up), (0, 255, 255), 10, cv2.LINE_AA)
        cv2.line(input_warped_, (cols_up[3], row_up), (cols_up[3], row_up), (0, 255, 255), 10, cv2.LINE_AA)
        cv2.line(input_warped_, (cols_up[4], row_up), (cols_up[4], row_up), (0, 255, 255), 10, cv2.LINE_AA)

        if self.parking_spot == -1:
            parking_spot1 = input_warped[:row_up,cols_up[1]:cols_up[2],:]
            parking_spot1_ = parking_spot1.copy()
            parking_spot1 = self.hsv(parking_spot1, color='purple')
            square_spot1, flag1 = self.detect_square(parking_spot1, self.min_area)
            if flag1:
                cv2.drawContours(parking_spot1_, [square_spot1], 0, (128, 0, 128), 3, cv2.LINE_AA)
            else:
                self.parking_spot = 1
                print('parking spot: 1')

            parking_spot2 = input_warped[:row_up,cols_up[2]:cols_up[3],:]
            parking_spot2_ = parking_spot2.copy()
            parking_spot2 = self.hsv(parking_spot2, color='purple')
            square_spot2, flag2 = self.detect_square(parking_spot2, self.min_area)
            if flag2:
                cv2.drawContours(parking_spot2_, [square_spot2], 0, (128, 0, 128), 3, cv2.LINE_AA)
            else:
                self.parking_spot = 2
                print('parking spot: 2')
            
            parking_spot3 = input_warped[:row_up,cols_up[3]:cols_up[4],:]
            parking_spot3_ = parking_spot3.copy()
            parking_spot3 = self.hsv(parking_spot3, color='purple')
            square_spot3, flag3 = self.detect_square(parking_spot3, self.min_area)
            if flag3:
                cv2.drawContours(parking_spot3_, [square_spot3], 0, (128, 0, 128), 3, cv2.LINE_AA)
            else:
                self.parking_spot = 3
                print('parking spot: 3')
            
            cv2.imshow('parking spot1', parking_spot1_)
            cv2.imshow('parking spot2', parking_spot2_)
            cv2.imshow('parking spot3', parking_spot3_)

            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        input_red = self.hsv(input_warped, color='red')  
        input_blue = self.hsv(input_warped, color='blue')  
        input_purple = self.hsv(input_warped, color='purple')
        
        square_red, _ = self.detect_square(input_red)
        square_blue, _ = self.detect_square(input_blue)
        
        center_red = np.int0(np.mean(square_red, axis=0))
        center_blue = np.int0(np.mean(square_blue, axis=0)) 
       
        cv2.drawContours(input_warped_, [square_red], 0, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.drawContours(input_warped_, [square_blue], 0, (255, 0, 0), 3, cv2.LINE_AA)
        
        cv2.arrowedLine(input_warped_, (center_blue[0], center_blue[1]), (center_red[0], center_red[1]), (0, 255, 0), 3, cv2.LINE_AA)
      
        x_pos = (center_blue[0] + center_red[0]) / 2 - cols_down
        x_pos = x_pos * self.xpix_ratio + 89
        y_pos = row_down - (center_blue[1] + center_red[1]) / 2
        y_pos = y_pos * self.ypix_ratio + 186

        heading = np.arctan2(center_blue[1] - center_red[1], center_red[0] - center_blue[0])

        print('X POS: %.2f / Y POS: %.2f / HEADING: %.4f' % (x_pos, y_pos, heading))

        pose_msg = Pose()
        pose_msg.position.x = x_pos
        pose_msg.position.y = y_pos
        pose_msg.position.z = heading
        pose_msg.orientation.x = self.parking_spot
        pose_msg.orientation.y = self.direction 
        pose_msg.orientation.z = self.is_stop 
        
        if self.xpix_ratio != None and self.ypix_ratio != None:
            self.pose_pub.publish(pose_msg)
        
        cv2.imshow('green', input_green)
        cv2.imshow('red', input_red)
        cv2.imshow('blue', input_blue)
        # cv2.imshow('parking space', input_)
        cv2.imshow('parking space warped', input_warped_)
        
        cv2.waitKey(1)

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
            mask = cv2.inRange(hsv, (115, 160, 50), (130, 255, 255))
        elif color == 'blue':
            mask = cv2.inRange(hsv, (10, 200, 50), (30, 255, 255))
        elif color == 'purple':
            # mask = cv2.inRange(hsv, (130, 60, 10), (180, 255, 255))
            mask = cv2.inRange(hsv, (130, 100, 10), (180, 255, 255))

        imask = mask > 0
        output = np.zeros_like(hsv, np.uint8)
        output[imask] = 255

        return output[:,:,0]
    
    def detect_square(self, input, min_area=0):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
        clean = cv2.morphologyEx(input, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
        clean = cv2.morphologyEx(input, cv2.MORPH_CLOSE, kernel)

        contours = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        square = None
        is_square = False
        area = 0

        for c in contours:
            rot_rect = cv2.minAreaRect(c)
            temp_area = rot_rect[1][0] * rot_rect[1][1]

            if area <= temp_area and temp_area >= min_area:
                square = cv2.boxPoints(rot_rect)
                square = np.int0(square)
                is_square = True
                area = temp_area

        return square, is_square

    def detect_lines(self, input, thres_up=100, thres_down=100, offset=150):
        mask = input == 255
        input_ = input.copy()
        input_[mask] = 1

        hist_col_up = np.sum(input_[0:540,:], axis=0)
        hist_col_down = np.sum(input_[540:1080,:], axis=0)
        hist_row = np.sum(input_[:,:], axis=1)

        row_up = np.argmax(hist_row[0:540])
        row_down = np.argmax(hist_row[540:1080]) + 540

        pos = 0
        
        cols_up = []
        cols_down = []

        while pos < 1320:
            if hist_col_up[pos] >= thres_up:
                cols_up.append(pos)
                pos = pos + offset
            else:
                pos = pos + 1

        pos = 0

        while pos < 1320:
            if hist_col_down[pos] >= thres_down:
                cols_down.append(pos)
                pos = pos + offset
            else:
                pos = pos + 1

        return row_up, row_down, cols_down[0], cols_up

if __name__ == '__main__':
    rospy.init_node('parking_node')
    r = rospy.Rate(10)
    p = Parking()

    rospy.spin()
