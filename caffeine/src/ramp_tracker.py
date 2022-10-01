#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class RampTracker:
    def __init__(self):
        # Ros Setting
        # Ros node init
        rospy.init_node('ramp_tracker')
        self.ros_rate = rospy.Rate(10)
        
        # publish control command
        self.pub_ctrl_motor = rospy.Publisher('ctrl_motor', Float32, queue_size=1)
        self.pub_ctrl_servo = rospy.Publisher('ctrl_servo', Float32, queue_size=1)

        # subscribe the image
        self.img_front_sub = rospy.Subscriber('/front_cam/image_raw', Image, self.img_front_callback)
        self.cv_bridge = CvBridge() # ros image massage를 사진으로 받아오는 함수
        
        # 초기화
        self.img_front = None

        self.is_img_front = False
        # self.is_left = False
        # self.is_right = False
        # self.is_back = False

        ## 테스트용 ##
        path = '/home/juntae/catkin_ws/src/caffeine/src/'
        file = 'front_ramp.jpg'
        img_front_array = np.fromfile(path+file, np.uint8)
        self.img_front = cv2.imdecode(img_front_array, cv2.IMREAD_COLOR)
        self.is_img_front = True

        # 카메라 변환
        self.src_front = np.float32([[625, 442],    # below right
                                     [0,   442],    # below left
                                     [149, 192],    # top   left
                                     [478, 192]])   # top   right
        
        self.dst_front = np.float32([[439, 460],   # below right
                                     [201, 460],   # below left
                                     [201, 192],    # top   left
                                     [439, 192]])   # top   right

        self.m_per_pix = 0.00252 # meters per pixel in x dimension

        ## Stanley Parameters
        # Calculate desired steering angle
        self.wheel_base = 0.51    #[m]
        self.gain_str = 1.0
        self.gain_cte = 0.01     

        ## 최종 motor, servo 입력값
        self.ros_input_motor = 0.0    # -255 ~ 255 (pwm duty)
        self.ros_input_servo = 0.0    # -20 ~ 20 deg

        self.count = 0
        # plt.ion()
        # self.plt_fig, self.plt_ax = plt.subplots(figsize = (10, 8))

    
    def process(self):
        print("start ramp_tracker")
        while not rospy.is_shutdown():
            self.count += 1
            if self.is_img_front:
                img_bin = self.threshold_hls(self.img_front)
                img_warped, M, Minv = self.warp_perspective(img_bin)
                img_warped_croped, leftx, lefty, rightx, righty = self.detect_lane_pixels(img_warped)
                
                left_polyfit_refined, right_polyfit_refined, left_fitx, right_fitx, ploty \
                = self.search_around_poly(img_warped_croped, leftx, lefty, rightx, righty, verbose=True)

                # Calculate Curvature
                # left_curverad_real, right_curverad_real = self.measure_curvature(ploty, left_polyfit_refined, right_polyfit_refined)
                # curverad_mean = (left_curverad_real + right_curverad_real)/2    #[m]

                # Calcualte deviation from center (cross track error)
                cross_track_error = self.measure_distance_from_center(left_fitx, right_fitx, ploty)

                left_theta, right_theta = self.measure_theta(ploty, left_polyfit_refined, right_polyfit_refined)
                
                # calculate stanley steering angle
                steer_ref = (left_theta + right_theta)/2
                steer_stanley = self.gain_str * steer_ref + np.arctan(self.gain_cte * cross_track_error)

                print(self.count)
                if self.count >= 10:
                    os.system('clear')
                    print("cross_track_error : {:.2f}m".format(cross_track_error))
                    # print("steer_ref :{:.2f} deg".format(steer_ref*180.0/np.pi))
                    print("left theta {:.2f}, right theta {:.2f}".format(left_theta*180/np.pi, right_theta*180/np.pi))
                    print("Stanely Steering Angle: gain_str*steer_ref + atan(gain_cte * cte)")
                    print("{:.2f} + {:.2f} = {:.2f} deg".format( \
                        self.gain_str * steer_ref*180/np.pi, \
                        np.arctan(self.gain_cte * cross_track_error)*180/np.pi, \
                        steer_stanley * 180/np.pi))                    

                    self.count = 0

                self.pub_ctrl_servo.publish(steer_stanley*180/np.pi)
                # self.pub_ctrl_motor.publish(self.ros_input_motor)

                # if self.plt_start == False:
                #     plt.show()
                #     self.plt_start = True
            self.ros_rate.sleep()



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
        s_thresh_min = 85
        s_thresh_max = 255
        # s_thresh_max = -1

        # Note: img is the undistorted, RGB image
        img_hls = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HLS)
        h_channel = img_hls[:,:,0]
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

        return combined_binary
        
    
    def warp_perspective(self, img_bin, verbose = False):
        '''
        # Function Description
        Convert perspective of binary image to bird's view image

        # Parameter
        img_bin               = binary, thresholded image
        verbose               = show both S channel image and undistorted image when verbose == True

        # Return
        combined_binary       = warped image
        '''
        # Image size in (x, y) direction
        img_size = (img_bin.shape[1], img_bin.shape[0])

        # Front original points(src) in image and destination of points(dst) in warped image
        src = self.src_front
        dst = self.dst_front
        
        # Perspective Transform Matrix(M) and Inverse transform matrix(Minv)
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)

        # Warp the image
        binary_warped = cv2.warpPerspective(img_bin, M, (img_size))

        ## Visualization ##
        if verbose == True:
            img_roi = np.dstack((img_bin, img_bin, img_bin)) * 255
            img_roi = cv2.polylines(img_roi, [np.int32(src)], True, (255, 0, 0), 2)
            img_warped = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
            img_warped = cv2.polylines(img_warped, [np.int32(dst)], True, (255, 0, 0), 2)

            # f, (ax1, ax2) = plt.subplots(1, 2, figsize = (32, 9))
            # ax1.set_title('Before Warping', fontsize = 30)
            # ax1.imshow(img_roi)
            # ax2.set_title('After Warping', fontsize = 30)
            # ax2.imshow(img_warped)
            
            cv2.imshow('warped image', img_warped)
            cv2.waitKey(1)

        return binary_warped, M, Minv

    def detect_lane_pixels(self, img_warped, verbose = False):
        # 가장 아래 80 pixel 제거
        img_warped_croped = img_warped[:400, :]
        # plt.imshow(img_warped_croped, cmap='gray')
        
        # Image 교정용 Rotation
        # img_warped_croped = rotate_image(img_warped_croped, 0, (0, 0))
        
        # HYPERPARAMETERS
        nwindows = 10           # Choose the number of sliding windows    
        margin = 50            # Set the width of the windows +/- margin
        minpix = 50            # Set minimum number of pixels found to recenter window

        # Set height of windows - based on nwindows above and image shape
        window_height = np.int(img_warped_croped.shape[0]//nwindows)

        # Take a histogram of the bottom half of the image
        histogram = np.sum(img_warped_croped[img_warped_croped.shape[0]//2:img_warped_croped.shape[0],:], axis=0)

        # Visualize the resulting histogram
        # plt.plot(histogram)
        # plt.show()

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Identify the x and y positions of all nonzero (i.e. activated) pixels in the image
        # nonzero
        nonzero = img_warped_croped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Current positions to be updated later for each window in nwindows
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []
        
        # [Visualization] Create an output image to draw on and visualize the result
        if verbose == True:
            out_img = np.dstack((img_warped_croped*255, img_warped_croped*255, img_warped_croped*255))
        
        # Step through the windows one by one
        # for window in range(nwindows):
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = img_warped_croped.shape[0] - (window+1)*window_height
            win_y_high = img_warped_croped.shape[0] - window*window_height
            #Find the four below boundaries of the window ###
            win_xleft_low = leftx_current - margin  # Update this
            win_xleft_high = leftx_current + margin  # Update this
            win_xright_low = rightx_current - margin  # Update this
            win_xright_high = rightx_current + margin  # Update this

            # [Visualization] Draw the windows on the visualization image
            if verbose == True:
                cv2.rectangle(out_img,(win_xleft_low, win_y_low),
                (win_xleft_high, win_y_high),(0,255,0), 2) 
                cv2.rectangle(out_img,(win_xright_low,win_y_low),
                (win_xright_high,win_y_high),(0,255,0), 2) 

            ### TO-DO: Identify the nonzero pixels in x and y within the window ###
            good_left_inds = ((nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high) & (nonzeroy >= win_y_low) & (nonzeroy < win_y_high)).nonzero()[0]
            good_right_inds = ((nonzerox >= win_xright_low) & (nonzerox < win_xright_high) & (nonzeroy >= win_y_low) & (nonzeroy < win_y_high)).nonzero()[0]

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            ### TO-DO: If you found > minpix pixels, recenter next window ###
            ### (`right` or `leftx_current`) on their peak histogram ###
            if(len(good_left_inds) > minpix):
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if(len(good_right_inds) > minpix):
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
                
        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        
        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        
        if verbose == True:
            cv2.imshow('detect lane pixel', out_img)
            cv2.waitKey(1)
        
        return img_warped_croped, leftx, lefty, rightx, righty

    def fit_poly(self, img_shape, leftx, lefty, rightx, righty):
        ### TO-DO: Fit a second order polynomial to each with np.polyfit() ###
        left_polyfit = np.polyfit(lefty, leftx, 2)
        right_polyfit = np.polyfit(righty, rightx, 2)
        # Generate x and y values for plotting
        ploty = np.linspace(0, img_shape[0]-1, img_shape[0])
        ### TO-DO: Calc both polynomials using ploty, left_fit and right_fit ###
        left_fitx = left_polyfit[0] * (ploty ** 2) + left_polyfit[1] * ploty + left_polyfit[2]
        right_fitx = right_polyfit[0] * (ploty ** 2) + right_polyfit[1] * ploty + right_polyfit[2]
        
        return left_polyfit, right_polyfit, left_fitx, right_fitx, ploty

    def search_around_poly(self, binary_warped, leftx, lefty, rightx, righty, verbose = False):
        # HYPERPARAMETER
        ## Choose the width of the margin around the previous polynomial to search
        margin = 10
        
        # polynomial fitting
        img_shape = binary_warped.shape
        left_polyfit, right_polyfit, _, _, _ = self.fit_poly(img_shape, leftx, lefty, rightx, righty)
        
        # Grab activated pixels
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        ### TO-DO: Set the area of search based on activated x-values ###
        ### within the +/- margin of our polynomial function ###
        ### Hint: consider the window areas for the similarly named variables ###
        ### in the previous quiz, but change the windows to our new search area ###
        left_lane_inds = ((nonzerox > (left_polyfit[0]*(nonzeroy**2) + left_polyfit[1]*nonzeroy + left_polyfit[2] - margin))
                        & (nonzerox < (left_polyfit[0]*(nonzeroy**2) + left_polyfit[1]*nonzeroy + left_polyfit[2] + margin)))
        right_lane_inds = ((nonzerox > (right_polyfit[0]*(nonzeroy**2) + right_polyfit[1]*nonzeroy + right_polyfit[2] - margin)) 
                        & (nonzerox < (right_polyfit[0]*(nonzeroy**2) + right_polyfit[1]*nonzeroy + right_polyfit[2] + margin)))
        
        # Again, extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Fit new polynomials
        left_polyfit_refined, right_polyfit_refined, left_fitx, right_fitx, ploty = \
            self.fit_poly(binary_warped.shape, leftx, lefty, rightx, righty)
        
        ## Visualization ##
        # Create an image to draw on and an image to show the selection window
        if verbose == True:
            out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
            window_img = np.zeros_like(out_img)
            # Color in left and right line pixels
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

            # Generate a polygon to illustrate the search window area
            # And recast the x and y points into usable format for cv2.fillPoly()
            left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
            left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin, 
                                    ploty])))])
            left_line_pts = np.hstack((left_line_window1, left_line_window2))
            right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
            right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, 
                                    ploty])))])
            right_line_pts = np.hstack((right_line_window1, right_line_window2))

            # Draw the lane onto the warped blank image
            cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
            cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
            result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

            # Plot the polynomial lines onto the image
            # plt.plot(left_fitx, ploty, color='yellow')
            # plt.plot(right_fitx, ploty, color='yellow')
            # plt.imshow(result)

            cv2.imshow('search around poly', result)
            cv2.waitKey(1)
        ## End visualization steps ##
        
        return left_polyfit_refined, right_polyfit_refined, left_fitx, right_fitx, ploty
    
    def measure_theta(self, ploty, left_fit, right_fit):
        '''
        Calculates the curvature of polynomial functions in pixels.
        '''
        # Hyper Parameter
        m_per_pix = self.m_per_pix
        
        # Define y-value where we want radius of curvature
        # We'll choose the 0.75 * maximum y-value, corresponding to the 75% bottom of the image
        y_eval = np.int(np.max(ploty) * 0.75)
        
        # Calculation of R_curve (radius of curvature)
        left_theta = np.arctan(2*left_fit[0]*y_eval + left_fit[1])
        right_theta = np.arctan(2*right_fit[0]*y_eval + right_fit[1])
        
        return left_theta, right_theta
    
    def measure_curvature(self, ploty, left_fit, right_fit):
        '''
        Calculates the curvature of polynomial functions in pixels.
        '''
        # Hyper Parameter
        m_per_pix = self.m_per_pix
        
        # Define y-value where we want radius of curvature
        # We'll choose the 0.75 * maximum y-value, corresponding to the 75% bottom of the image
        y_eval = np.int(np.max(ploty) * 0.75)
        
        # Calculation of R_curve (radius of curvature)
        left_curverad = ((1 + (2*left_fit[0]*y_eval + left_fit[1])**2)**1.5) / (2*left_fit[0])
        right_curverad = ((1 + (2*right_fit[0]*y_eval + right_fit[1])**2)**1.5) / (2*right_fit[0])
        
        left_curverad_real, right_curverad_real = (left_curverad* m_per_pix, right_curverad* m_per_pix) 
        
        return left_curverad_real, right_curverad_real

    def measure_distance_from_center(self, left_fitx, right_fitx, ploty):
        # Define conversions in x and y from pixels space to meters
        xm_per_pix = self.m_per_pix
        y_eval_pos_ratio = 0.9
        
        center_fitx = (320 - (left_fitx + right_fitx) / 2) * xm_per_pix
        center_fit = np.array([center_fitx, ploty]).T
        
        # 아래로 75% 위치의 center로 부터의 거리 측정
        y_eval = np.int(np.max(ploty) * y_eval_pos_ratio)
        
        return center_fit[y_eval, 0]

    def img_front_callback(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
        self.img_front = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.is_img_front = True

    @staticmethod
    def rotate_image(image, angle, translation):
        ''' rotate image
        @params
        image : image
        angle : angle to rotate [deg]
        translation : move image
        
        @return
        new_image : rotated_image
        '''
        row,col = image.shape[:2]
        center=tuple(np.array([row,col])/2)
        rot_mat = cv2.getRotationMatrix2D(center,angle,1.0)   # center, angle[deg], ratio
        aff_mat = rot_mat.copy()
        aff_mat[:, 2] = translation
        image_rotated = cv2.warpAffine(image, aff_mat, (col,row))
        return image_rotated
    
            
if __name__ == '__main__':
    ramp_tracker = RampTracker()
    ramp_tracker.process()