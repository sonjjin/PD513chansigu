#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import csv
import time

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class Ramptracker:
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.img_front_sub = rospy.Subscriber('/front_cam/image_raw', Image, self.img_front_callback)
        self.img_left_sub = rospy.Subscriber('/left_cam/image_raw', Image, self.img_left_callback)
        self.img_right_sub = rospy.Subscriber('/right_cam/image_raw', Image, self.img_right_callback)
        self.img_back_sub = rospy.Subscriber('/rear_cam/image_raw', Image, self.img_back_callback)

        self.sub_cur_speed = rospy.Subscriber('/arduino_ctrl/ctrl_motor', Float32, self.callback_speed)
        self.sub_cur_steer = rospy.Subscriber('/arduino_ctrl/ctrl_servo', Float32, self.callback_steer)

        self.pub_ctrl_servo = rospy.Publisher('/arduino_ctrl/ctrl_servo', Float32, queue_size=1)
        self.pub_ctrl_motor = rospy.Publisher('/arduino_ctrl/ctrl_motor', Float32, queue_size=1)

        self.is_front = False
        self.is_left = False
        self.is_right = False
        self.is_back = False
        
        self.cur_img_front = None
        self.cur_img_left = None
        self.cur_img_right = None
        self.cur_img_back = None

        self.is_cur_speed = False
        self.cur_speed = None
        self.is_cur_steer = False
        self.cur_steer = None
        self.turn_right = True # turn right: true, turn left: false
        self.mode = 0 # 0: ramp tracking, 1: parking
        ## 테스트용 ##
        # path = '/home/juntae/catkin_ws/src/caffeine/src/seq/front/'
        # file = '00.png'
        # img_front_array = np.fromfile(path+file, np.uint8)
        # self.cur_img_front = cv2.imdecode(img_front_array, cv2.IMREAD_COLOR)
        # self.is_front = True

        self.forward_src = np.float32([
                    (125, 180),
                    (0, 440),
                    (500, 180),
                    (640, 440)
                ])

        self.left_src = np.float32([    
                    (100, 45),  
                    (5,415),    
                    (510, 45),  
                    (620, 410)  
                ])  
    
        self.right_src = np.float32([
                    (100, 45),
                    (5,415),
                    (510, 45),
                    (620, 410)
                ])

        self.forward_dst = np.float32([
                    (150, 90),
                    (170, 440),
                    (560, 90),
                    (470, 445)
                ])    

        self.left_dst = np.float32([
                    (140, 60),
                    (140, 460),
                    (480, 65),
                    (480, 445)
                ])
                
        self.right_dst = np.float32([
                    (140, 60),
                    (140, 450),
                    (480, 65),
                    (480, 460)
                ])
    
        self.control_state = 0

    # lane check
        self.check_left = False
        self.check_right = False

        self.left_fit = None
        self.right_fit = None

        self.m_per_pixel = 0.00252
        self.steer = None

        self.count = 0
        self.iter = 0


        self.start_time = time.time()

    '''
    image callback
    '''
    def img_front_callback(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
        self.cur_img_front = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # print("front",  self.cur_img_front.dtype) 
        self.is_front = True

    def img_left_callback(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
        self.cur_img_left = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        # print("left", self.cur_img_left.dtype) 
        self.is_left = True
    
    def img_right_callback(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
        self.cur_img_right = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        # print("right", self.cur_img_right.dtype) 
        self.is_right = True
    
    def img_back_callback(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
        self.cur_img_back = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        # print("rear", self.cur_img_back.dtype) 
        self.is_back = True
    
    def callback_speed(self, data):
        if not self.is_cur_speed:
            self.cur_speed = data.data
            self.is_cur_speed = True
            
    def callback_steer(self, data):
        if not self.is_cur_steer:
            self.cur_steer = data.data
            self.is_cur_steer = True
    '''
    hsv and image detetion
    '''
    def image_clean(self, input):
        H, W = input.shape[:2]
        # using morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
        clean = cv2.morphologyEx(input, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
        img_clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)
        
        return img_clean

    def hsv(self, img, color='yellow'):

        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)

        if color == 'green':
            mask = cv2.inRange(hsv, (25, 60, 50), (86, 255, 255))
        elif color == 'red':
            mask = cv2.inRange(hsv, (115, 100, 50), (130, 255, 255))
        elif color == 'blue':
            mask = cv2.inRange(hsv, (10, 150, 50), (30, 255, 255))
        elif color == 'yellow':
            mask = cv2.inRange(hsv, (40, 60, 80), (160, 255, 255))
        elif color == 'black':
            mask = cv2.inRange(hls, (0, 0, 0), (180, 100, 255))
        
        imask = mask > 0
        temp = np.zeros_like(hsv, np.uint8)
        temp[imask] = 255    
        output = self.image_clean(temp[:,:,0])
        # plt.imshow(cv2.cvtColor(output, cv2.COLOR_BGR2RGB))

        return output
    
    '''
    image wrapping
    '''

    def front(self, img):
        IMAGE_H, IMAGE_W, _ = img.shape
        
        src = self.forward_src
        dst = self.forward_dst
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H))
        output = warped_img[90:,:-10]
        return output
    
    def side_left(self, img):
        IMAGE_H, IMAGE_W, _ = img.shape

        src = self.left_src
        dst = self.left_dst
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation.mkv
        
        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
        output = warped_img[90:,:]
        # output[left_shift:,:] = output[:-left_shift,:]
        output = cv2.rotate(output, cv2.ROTATE_90_COUNTERCLOCKWISE)#[:,:350]
        # warped_img = cv2.warpPerspective(img, M, (IMAGE_H, IMAGE_W)) # Image warping
        
        return output
        
    def side_right(self, img):
        IMAGE_H, IMAGE_W, _ = img.shape

        src = self.right_src
        dst = self.right_dst
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation.mkv

        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
        output = warped_img[90:,:]
        output = cv2.rotate(output, cv2.ROTATE_90_CLOCKWISE)#[:,:350]
        # warped_img = cv2.warpPerspective(img, M, (IMAGE_H, IMAGE_W)) # Image warping
        return output 
        
    '''
    Stop lane detection 
    input: warp image
    '''
    def stop_lane(self, img):
        img_hsv = self.hsv(img, 'yellow')
        img_bin = img_hsv
        # out_img = cv2.cvtColor(img_bin, cv2.COLOR_GRAY2BGR)
        
        nwindows = 10
        margin = 20
        minpix = 50

        window_width = np.int64(img_bin.shape[1]//nwindows)

        # Take a histogram of the bottom half of the image
        histogram = np.sum(img_bin[:,:img_bin.shape[1]-np.int32(img_bin.shape[1]/2)], axis=1)
        his_max = np.max(histogram)
        # print(his_max)
        if his_max < 10000:
            return

        # print(np.max(histogram))
        # cv2.imwrite('p.png', img_bin[:,0:img_bin.shape[1]-np.int32(img_bin.shape[1]/2)])
        top_basey = np.argmax(histogram[:])

        nonzero = img_bin.nonzero()
        nonzeroy = np.array(nonzero[1])
        nonzerox = np.array(nonzero[0])

        # Current positions to be updated later for each window in nwindows
        top_current = top_basey
        top_lane_inds = []

        for window in range(nwindows):

            win_x_low = img_bin.shape[1] - (window+1)*window_width
            win_x_high = img_bin.shape[1] - window*window_width

            win_top_low = top_current - margin
            win_top_high = top_current + margin 

            good_top_inds = ((nonzerox >= win_top_low) & (nonzerox < win_top_high) & (nonzeroy >= win_x_low) & (nonzeroy < win_x_high)).nonzero()[0]
            top_lane_inds.append(good_top_inds)


            if(len(good_top_inds) > minpix):
                top_current = np.int32(np.mean(nonzerox[good_top_inds]))
                
        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        top_lane_inds = np.concatenate(top_lane_inds)
        # bottom_lane_inds = np.concatenate(bottom_lane_inds)

        # Extract left and right line pixel positions
        topx = nonzerox[top_lane_inds]
        topy = nonzeroy[top_lane_inds] 
        # bottomx = nonzerox[bottom_lane_inds]
        # bottomy = nonzeroy[bottom_lane_inds]
        ### TO-DO: Fit a second order polynomial to each using `np.polyfit` ###
        top_fit = np.polyfit(topy, topx, 1)
        # bottom_fit = np.polyfit(bottomy, bottomx, 1)
        # print(bottom_fit)

        # plotx = np.linspace(0, img_bin.shape[1]-1, img_bin.shape[1])
        # # print(plotx)
        # try:
        #     top_fity = top_fit[0]*plotx + top_fit[1]
        #     # bottom_fity = bottom_fit[0]*plotx + bottom_fit[1]
        # except TypeError:
        #     # Avoids an error if `left` and `right_fit` are still none or incorrect
        #     print('The function failed to fit a line!')
        #     top_fity = 1*plotx**2 + 1*plotx
            # bottom_fity = 1*plotx**2 + 1*plotx
        ref_x = np.int32(img_bin.shape[1]/2)
        ref_y = np.int32(top_fit[0]*ref_x + top_fit[1])
        print(ref_y)
        if ref_y > 330:
            self.control_state = 1
            
        # ## Visualization ##
        # # Colors in the left and right lane regions
        # out_img[topx, topy] = [255, 0, 0]
        

        # plt.plot(plotx, top_fity, color='yellow')
        # # plt.plot(plotx, bottom_fity, color='yellow')
        # plt.imshow(out_img) 
    
    
    
    '''
    lane detection 
    input: warp image
    '''
    def lane_detect(self, img, fit):
        self.check_left = False
        self.check_right = False

        img_hsv = self.hsv(img, 'yellow')
        img_bin = img_hsv
        img_rgb = np.dstack((img_bin, img_bin, img_bin))

        is_there_lane = False
        img_sum = np.sum(img_bin)

        if img_sum > 0:
            is_there_lane = True

        if is_there_lane:
            # HYPERPARAMETERS
            nwindows = 10 # Choose the number of sliding windows
            margin = 50 # Set the width of the windows +/- margin
            minpix = 50 # Set minimum number of pixels found to recenter window
            window_height = np.int64(img_rgb.shape[0]//nwindows) # Set height of windows - based on nwindows above and image shape

            histogram = np.sum(img_bin[200:], axis=0)

            # find lane
            midpoint = np.int64(histogram.shape[0]//2) - 100
            leftx_base = np.argmax(histogram[:midpoint])
            if leftx_base != 0:
                self.check_left = True
            rightx_base = np.argmax(histogram[midpoint:])
            if rightx_base != 0:
                rightx_base = rightx_base + midpoint
                self.check_right = True

            # find nonzero value in out_img
            nonzero = img_rgb.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])

            # Current positions to be updated later for each window in nwindows
            leftx_current = leftx_base
            rightx_current = rightx_base

            # Create empty lists to receive left and right lane pixel indices
            left_lane_inds = []
            right_lane_inds = []

            # Step through the windows one by one
            # for window in range(nwindows):
            for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
                win_y_low = img_rgb.shape[0] - (window+1)*window_height
                win_y_high = img_rgb.shape[0] - window*window_height
                # print(win_y_low, win_y_high)
                
                if self.check_left == True:
                    win_xleft_low = leftx_current - margin  # Update this
                    win_xleft_high = leftx_current + margin  # Update this
                    # cv2.rectangle(img_rgb,(win_xleft_low, win_y_low),(win_xleft_high, win_y_high),(204,0,255), 2) 
                    good_left_inds = ((nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high) & (nonzeroy >= win_y_low) & (nonzeroy < win_y_high)).nonzero()[0]
                    left_lane_inds.append(good_left_inds)
                    if(len(good_left_inds) > minpix):
                        leftx_current = np.int64(np.mean(nonzerox[good_left_inds]))
                ### TO-DO: Find the four below boundaries of the window ###
                if self.check_right == True:
                    win_xright_low = rightx_current - margin  # Update this
                    win_xright_high = rightx_current + margin  # Update this
                    # cv2.rectangle(img_rgb,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(51,153,255), 2) 
                    good_right_inds = ((nonzerox >= win_xright_low) & (nonzerox < win_xright_high) & (nonzeroy >= win_y_low) & (nonzeroy < win_y_high)).nonzero()[0]
                    right_lane_inds.append(good_right_inds)
                    if(len(good_right_inds) > minpix):
                        rightx_current = np.int64(np.mean(nonzerox[good_right_inds]))
                
                    
            if self.check_left == True:
                left_lane_inds = np.concatenate(left_lane_inds)
                leftx = nonzerox[left_lane_inds]
                lefty = nonzeroy[left_lane_inds]
                left_fit = np.polyfit(lefty, leftx, fit)
                self.left_fit = left_fit
                
                
            if self.check_right == True:
                right_lane_inds = np.concatenate(right_lane_inds)
                rightx = nonzerox[right_lane_inds]
                righty = nonzeroy[right_lane_inds]
                right_fit = np.polyfit(righty, rightx, fit)
                self.right_fit = right_fit

            # Generate x and y values for plotting
            ploty = np.linspace(0, img_rgb.shape[0]-1, img_rgb.shape[0]) # y value
            # print(ploty)
            if fit == 2:
                try:
                    if self.check_left == True:
                        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
                    if self.check_right == True:
                        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
                except TypeError:
                    # Avoids an error if `left` and `right_fit` are still none or incorrect
                    print('The function failed to fit a line!')
                    left_fitx = 1*ploty**2 + 1*ploty
                    right_fitx = 1*ploty**2 + 1*ploty

                ## Visualization ##
                # Colors in the left and right lane regions
                if self.check_left == True:
                    img_rgb[lefty, leftx] = [255, 0, 0]
                    left_fit_idx = np.stack((left_fitx, ploty), axis = -1).astype(int)
                    img_rgb = cv2.polylines(img_rgb, [left_fit_idx] , False, (0, 255, 255), 3)

                    # plt.plot(left_fitx, ploty, color='green')
                    
                if self.check_right == True:   
                    img_rgb[righty, rightx] = [0, 0, 255]   
                    right_fit_idx = np.stack((right_fitx, ploty), axis = -1).astype(int)
                    img_rgb = cv2.polylines(img_rgb, [right_fit_idx] , False, (0, 255, 255), 3)

                    # plt.plot(right_fitx, ploty, color='yellow')
            
            img_viz = np.vstack((img, img_rgb))
            # cv2.imwrite('../results/seq/' + str(self.count) + '.png', img_rgb)
            # plt.savefig('../results/seq/' + str(self.count) + '.png')

            # plt.imshow(img)        
            # plt.clf() 
            
            self.count += 1   
            # self.check_left = False
            # self.check_right = False
            check_right = self.check_right
            check_left = self.check_left
            right_fit = self.right_fit
            left_fit = self.left_fit

        else:
            check_right = False
            check_left = False
            right_fit = np.zeros(fit)
            left_fit = np.zeros(fit)
            img_viz = np.vstack((img, img_rgb))

        return check_right, check_left, right_fit, left_fit, img_viz
        
    
    def get_steer(self, img_f, check_right_f, check_left_f, right_fit_f, left_fit_f, img_l, check_right_l, right_fit_l, img_r, check_left_r, left_fit_r):
        img_f_h = img_f.shape[0]/2
        img_l_h = img_l.shape[0]/2
        img_r_h = img_r.shape[0]/2
        f_h_target = img_f_h * 0.7
        l_h_target = img_l_h * 0.5
        r_h_target = img_r_h * 0.5

        left_sidelane = -100
        right_sidelane = -100
        # print(img.shape, h_target)
        
        if check_left_f:
            # left_fit = left_fit
            left_lane = left_fit_f[0]*f_h_target**2 + left_fit_f[1]*f_h_target + left_fit_f[2]
            left_curverad = ((1 + (2*left_fit_f[0]*f_h_target + left_fit_f[1])**2)**1.5) / (2*left_fit_f[0]) * self.m_per_pixel
            # print('left lane :', left_lane, left_curverad)
        
        if check_right_f:
            # right_fit = right_fit
            right_lane = right_fit_f[0]*f_h_target**2 + right_fit_f[1]*f_h_target + right_fit_f[2]
            right_curverad = ((1 + (2*right_fit_f[0]*f_h_target + right_fit_f[1])**2)**1.5) / (2*right_fit_f[0]) * self.m_per_pixel
            # print('right lane :', right_lane, right_curverad)

        if check_right_l:
            # right_fit = right_fit
            # left_sidelane = right_fit_l[0]*l_h_target**2 + right_fit_l[1]*l_h_target + right_fit_l[2]
            left_sidelane = right_fit_l[0]*l_h_target + right_fit_l[1]
            # right_curverad = ((1 + (2*right_fit_f[0]*h_target + right_fit_f[1])**2)**1.5) / (2*right_fit_f[0]) * self.m_per_pixel
            # print('right lane :', right_lane, right_curverad)
        if check_left_r:
            # right_fit = right_fit
            # left_sidelane = right_fit_l[0]*l_h_target**2 + right_fit_l[1]*l_h_target + right_fit_l[2]
            right_sidelane = left_fit_r[0]*r_h_target + left_fit_r[1]
            # right_curverad = ((1 + (2*right_fit_f[0]*h_target + right_fit_f[1])**2)**1.5) / (2*right_fit_f[0]) * self.m_per_pixel
            # print('right lane :', right_lane, right_curverad)
        
        ## Pseudo Stanley 오른쪽 차선 기준
        if check_right_f:
            frl_ref = 470
            ls_ref = 390
            rs_ref = 90
            cte = (frl_ref-right_lane)  # 높을수록 붙어서감
            gain_cte = 0.3      # 높을수록 민감
            gain_curv = -1      # 높을수록 민감
            gain_cte_l = 0.1
            gain_cte_r = 0.1
            steer = 0.0
            cte_l = 0
            cte_r = 0

            if check_right_l:
                cte_l = (ls_ref - left_sidelane)
                if cte_l < 60:
                    steer = gain_cte * cte + gain_curv / right_curverad - cte_l*gain_cte_l
                    steer = max(min(steer, 20.0), -20.0)
                    # self.steer = steer

                else:
                    steer = gain_cte * cte + gain_curv / right_curverad
                    steer = max(min(steer, 20.0), -20.0)
                    self.steer = steer

                if check_left_r:
                    cte_r = (rs_ref - right_sidelane)
                    if cte_r < 60:
                        steer = steer + cte_r*gain_cte_r
                        steer = max(min(steer, 20.0), -20.0)
                        self.steer = steer

            else:
                if check_left_r:
                    steer = gain_cte * cte + gain_curv / right_curverad
                    steer = max(min(steer, 20.0), -20.0)
                    self.steer = steer

                    cte_r = (rs_ref - right_sidelane)
                    if cte_r < 60:
                        steer = steer + cte_r*gain_cte_r
                        steer = max(min(steer, 20.0), -20.0)
                        self.steer = steer
                else:
                    steer = gain_cte * cte + gain_curv / right_curverad
                    steer = max(min(steer, 20.0), -20.0)
                    self.steer = steer


            # print("-----------------------")
            # print(' cte  +  curv')
            # print('{:.3} + {:.3}'.format(gain_cte * cte, gain_curv / right_curverad))
            # print('cte_l')
            # if check_right_l:
            #     print('{:.3}'.format(cte_l))
            #     print(left_sidelane)
            # else:
            #     print('None')
            #     print(left_sidelane)

            # print('cte_r')
            # if check_left_r:
            #     print('{:.3}'.format(cte_r))
            #     print(right_sidelane)
            # else:
            #     print('None')
            #     print(right_sidelane)

            # print('steer: {:.3}'.format(steer))
            # print("-----------------------")
            # print("\n")
            # ll = [str(self.steer), str(cte), str(cte_l), str(cte_r)]
           


            self.is_front = False
            self.is_left = False
            self.is_right = False
    
            return self.steer
        else:
            return None
    
    def ramp_process(self):
        if self.is_front and self.is_left and self.is_right:
            self.start_time = time.time()
            
            img_f = self.cur_img_front
            img_l = self.cur_img_left
            img_r = self.cur_img_right
            img_b = self.cur_img_back
            img_l = self.side_left(img_l)
            img_r = self.side_right(img_r)
            img_f = self.front(img_f)
            
            if self.control_state == 0:
                check_right_f, check_left_f, right_fit_f, left_fit_f, img_f_lane = self.lane_detect(img_f ,2)
                check_right_l, check_left_l, right_fit_l, left_fit_l, img_l_lane = self.lane_detect(img_l ,1)
                check_right_r, check_left_r, right_fit_r, left_fit_r, img_r_lane = self.lane_detect(img_r ,1)
                self.get_steer(
                    img_f_lane, check_right_f, check_left_f, right_fit_f, left_fit_f,
                    img_l_lane, check_right_l, right_fit_l,
                    img_r_lane, check_left_r, left_fit_r)
                self.stop_lane(img_f)
                cv2.imshow("lane_detection", img_f_lane)
                cv2.imshow("lane_detection_left", cv2.resize(img_l_lane, dsize=(300,500)))
                cv2.imshow("lane_detection_right", cv2.resize(img_r_lane, dsize=(300,500)))
                cv2.waitKey(1)
            if self.control_state == 1:
                self.steer = self.cur_steer
                self.iter += 1
            if self.iter > 15:
                self.control_state = 2
                # self.pub_ctrl_motor.publish(0)
            
        if self.is_front and self.is_right:
            img_b = self.cur_img_right
            img_b = self.side_right(img_b)
            # cv2.imshow('left', img_l)
            # print(img_l.shape)
            # print(self.count)

        if self.steer is not None:
            self.pub_ctrl_servo.publish(self.steer)

        # dt = time.time()-self.start_time
        # print('time :', dt)

        # if  self.iter == 0:
        #     with open('./time.csv', 'w') as f:
        #         wr = csv.writer(f)
        #         wr.writerow([dt])
        # else:    
        #     with open('./time.csv', 'a') as f:
        #         wr = csv.writer(f)
        #         wr.writerow([dt])
        print("-----------------------")
        print('goal distance: None')
        print('turn point distance: None')
        print('speed: {:.3}'.format(self.cur_speed))
        print('steer: {:.3}'.format(self.cur_steer))
        print('control state: ramp')
        print("-----------------------")
        
        self.is_cur_speed = False
        self.is_cur_steer = False
        
        return self.control_state
     
        
if __name__ == '__main__':
    rospy.init_node('lane_detection')
    r = rospy.Rate(10)
    rt = Ramptracker()
    c = 0
    while not rospy.is_shutdown():
        if c == 0:      
            c = rt.ramp_process()
            r.sleep()
        # print(c)
    
    rospy.spin()
        
        
    
    
