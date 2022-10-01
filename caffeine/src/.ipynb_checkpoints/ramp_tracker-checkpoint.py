#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import matplotlib
import numpy as np
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import PyQt5

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class RampTracker:
    def __init__(self):
        # publish control command
        self.pub_ctrl_motor = rospy.Publisher('ctrl_motor', Float32, queue_size=1)
        self.pub_ctrl_servo = rospy.Publisher('ctrl_servo', Float32, queue_size=1)
        
        self.ros_input_motor = 0.0    # -255 ~ 255 (pwm duty)
        self.ros_input_servo = 0.0    # -20 ~ 20 deg
        
        # subscribe the image
        self.cv_bridge = CvBridge() # ros image massage를 사진으로 받아오는 함수
        self.img_front_sub = rospy.Subscriber('/front_cam/image_raw', Image, self.img_front_callback)
        
        # 초기화
        # self.cur_img_front = None
        ## 테스트용 ##
        path = '/home/juntae/catkin_ws/src/caffeine/src/'
        file = 'front_ramp.jpg'
        img_front_array = np.fromfile(path+file, np.uint8)
        self.img_front = cv2.imdecode(img_front_array, cv2.IMREAD_COLOR)
        
        self.is_img_front = False
        # self.is_left = False
        # self.is_right = False
        # self.is_back = False
        
        self.src_front = np.float32([[625, 442],    # below right
                                     [0,   442],    # below left
                                     [149, 192],    # top   left
                                     [478, 192]])   # top   right
        
        self.dst_front = np.float32([[439, 460],   # below right
                                     [201, 460],   # below left
                                     [201, 192],    # top   left
                                     [439, 192]])   # top   right
    
    def process(self):
        img_bin = self.threshold_hls(self.img_front, verbose=False)
        self.warp_perspective(img_bin, verbose=True)
        
        self.pub_ctrl_motor.publish(self.ros_input_motor)
        self.pub_ctrl_servo.publish(self.ros_input_servo)
    
    def threshold_hls(self, img_rgb, verbose=False):
        '''
        # Function Description
        Saturation Thresholding and binarization in HLS space

        # Parameter
        img_rgb               = RGB, undistorted image
        verbose               = show both S channel image and undistorted image when verbose == True

        # Return
        combined_binary       = binarized image thresholded in Lightness & Saturation channel of HLS space 
        '''

        # Threshold light channel for black
        l_thresh_min = 0
        # l_thresh_max = 120
        l_thresh_max = -1

        # Threshold color channel
        s_thresh_min = 60
        s_thresh_max = 255

        # Note: img is the undistorted, RGB image
        img_hls = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HLS)
        l_channel = img_hls[:,:,1]
        s_channel = img_hls[:,:,2]

        # Threshold color channel
        l_binary = np.zeros_like(l_channel, dtype = np.uint8)
        l_binary[(l_channel >= l_thresh_min) & (l_channel <= l_thresh_max)] = 1

        # Threshold color channel
        s_binary = np.zeros_like(s_channel, dtype = np.uint8)
        s_binary[(s_channel >= s_thresh_min) & (s_channel <= s_thresh_max)] = 1

        # Combined binary image
        combined_binary = np.zeros_like(l_binary, dtype = np.uint8)
        combined_binary[(l_binary == 1) | (s_binary == 1)] = 1

        if verbose == True:
            cv2.imshow('combined_img', combined_binary*255)
            cv2.waitKey(1)
            # cv2.destroyAllWindows()

        return combined_binary
        
    
    def warp_perspective(self, img_bin, **kwargs):
        '''
        # Function Description
        Convert perspective of binary image to bird's view image

        # Parameter
        img_bin               = binary, thresholded image
        **kwargs              = keyword arguments

        # kwargs
        verbose               = show both S channel image and undistorted image when verbose == True

        # Return
        combined_binary       = warped image
        '''
        # Image size in (x, y) direction
        img_size = (img_bin.shape[1], img_bin.shape[0])

        # Height(h), Width(w) of image
        h, w = img_bin.shape[:2]

        # Front original points(src) in image and destination of points(dst) in warped image
        src = self.src_front
        dst = self.dst_front
        
        # Perspective Transform Matrix(M) and Inverse transform matrix(Minv)
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)

        # Warp the image
        binary_warped = cv2.warpPerspective(img_bin, M, (img_size))

        ## Visualization ##
        for key, value in kwargs.items():
            if key == 'verbose' and value == True:
                img_roi = np.dstack((img_bin, img_bin, img_bin)) * 255
                img_roi = cv2.polylines(img_roi, [np.int32(src)], True, (255, 0, 0), 2)
                img_warped = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
                img_warped = cv2.polylines(img_warped, [np.int32(dst)], True, (255, 0, 0), 2)

                f, (ax1, ax2) = plt.subplots(1, 2, figsize = (32, 9))
                ax1.set_title('Before Warping', fontsize = 30)
                ax1.imshow(img_roi)
                ax2.set_title('After Warping', fontsize = 30)
                ax2.imshow(img_warped)
        return binary_warped, M, Minv

    def img_front_callback(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
        self.img_front = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.is_img_front = True
    
            
if __name__ == '__main__':
    rospy.init_node('ramp_tracking')
    ros_rate = rospy.Rate(30)
    ramp_tracker = RampTracker()
    ramp_tracker.process()
    
    while not rospy.is_shutdown():
        # ramp_tracker.process()
        ros_rate.sleep()
    rospy.spin()