#!/usr/bin/env python
#-*- coding: utf-8 -*-

import cv2
import cv2 as cv
import numpy as np

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SurroundView:
    def __init__(self):
        # subscribe the image
        self.cv_bridge = CvBridge() # ros image massage를 사진으로 받아오는 함수
        self.img_front_sub = rospy.Subscriber('/front_cam/image_raw', Image, self.img_front_callback)
        self.img_back_sub = rospy.Subscriber('/rear_cam/image_raw', Image, self.img_back_callback)
        self.img_left_sub = rospy.Subscriber('/left_cam/image_raw', Image, self.img_left_callback)
        self.img_right_sub = rospy.Subscriber('/right_cam/image_raw', Image, self.img_right_callback)
        # self.cur_img_front = cv2.imread('front.jpg', cv2.IMREAD_COLOR)
        # self.cur_img_left = cv2.imread('left.jpg', cv2.IMREAD_COLOR)
        # self.cur_img_right = cv2.imread('right.jpg', cv2.IMREAD_COLOR)
        # self.cur_img_back = cv2.imread('rear.jpg', cv2.IMREAD_COLOR)

        # 초기화

        self.iter = 0
        self.save = 0
        
        self.cur_img_front = None
        self.cur_img_left = None
        self.cur_img_right = None
        self.cur_img_back = None
        
        self.old_frame = None
        self.old_gray = None

        self.is_first = True
        
        self.is_front = False
        self.is_left = False
        self.is_right = False
        self.is_back = False
        
        # self.is_front = True
        # self.is_left = True
        # self.is_right = True
        # self.is_back = True


        # 이부분이 먼지 모르겠음 ㅋㅋ
        # 앞뒤 카메라 사각형 4점 좌표
        
        self.forward_src = np.float32([
            (125, 180),
            (0, 440),
            (500, 180),
            (640, 440)
        ])

        self.backward_src = np.float32([
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

        # self.forward_dst = np.float32([
        #     (70, 90),
        #     (170, 440),
        #     (530, 90),
        #     (470, 445)
        # ])

        self.forward_dst = np.float32([
            (150, 90),
            (170, 440),
            (560, 90),
            (470, 445)
        ])

        # self.backward_dst = self.forward_dst
        self.backward_dst = np.float32([
            (90, 90),
            (180, 440),
            (530, 85),
            (460, 445)
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

        self.right_shift = 25
        self.left_shift = 10
        
        self.contours = 0
        self.chk_contours = -99
        
        # 자동차 이미지 불러오는 것 인듯
        #self.car = cv2.imread('./car.jpg')
        self.car = cv2.imread('/home/juntae/catkin_ws/src/caffeine/src/car.png', cv2.IMREAD_COLOR)
        self.car = cv2.rotate(self.car, cv2.ROTATE_180)
        # car_final = cv2.resize(self.car, (910, 592), interpolation=cv2.INTER_LINEAR)
 
        self.car_width = 650
        self.car_height = 640+170
        self.car_final = cv2.resize(self.car, (self.car_width, self.car_height), interpolation=cv2.INTER_LINEAR)
        self.head_H = 0


        self.feature_params = dict(maxCorners = 30, 
                                   qualityLevel = 0.001,
                                    minDistance = 7,
                                    blockSize = 7
                                    )

        self.lk_params = dict(winSize = (5,5),
		maxLevel = 2,
		criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
        #self.final_car = cv2.resize(self.car, dsize=(420, 700),interpolation=cv2.INTER_LINEAR)

    # callback function
    def img_front_callback(self, data):
        if not self.is_front:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.cur_img_front = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_front = True

    def img_left_callback(self, data):
        if not self.is_left:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
            self.cur_img_left = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
            # print("left", self.cur_img_left.dtype) 
            self.is_left = True
    
    def img_right_callback(self, data):
        if not self.is_right:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
            self.cur_img_right = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
            # print("right", self.cur_img_right.dtype) 
            self.is_right = True
    
    def img_back_callback(self, data):
        if not self.is_back:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
            self.cur_img_back = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
            # print("rear", self.cur_img_back.dtype) 
            self.is_back = True

    # front ~ side_right bird eye view로 바꾸는거
    def front(self, img):
        IMAGE_H, IMAGE_W, _ = img.shape
        
        src = self.forward_src#np.float32([[249, 399], [549, 399], [289, 0], [509, 0]])
        dst = self.forward_dst#np.float32([[279, 399], [519, 399], [0, 0], [799, 0]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H))#[:300] # Image warping
        output = warped_img[90:,:-10]
        return output#cv2.resize(warped_img[200:,100:-100], dsize=(800, 400),interpolation=cv2.INTER_LINEAR)#warped_img

    def rear(self, img):
        IMAGE_H, IMAGE_W, _ = img.shape
    
        src = self.backward_src
        dst = self.backward_dst
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation
    
        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H))#[:300] # Image warping
        output = warped_img[90:,:]
        output = cv2.rotate(output, cv2.ROTATE_180)
        return output#cv2.resize(warped_img[200:,100:-100], dsize=(800, 400),interpolation=cv2.INTER_LINEAR)#warped_img
        
    def side_left(self, img):
        IMAGE_H, IMAGE_W, _ = img.shape

        src = self.left_src
        dst = self.left_dst
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation.mkv
        
        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
        output = warped_img[90:,:]
        output[self.left_shift:,:] = output[:-self.left_shift,:]
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
        output[self.right_shift:,:] = output[:-self.right_shift,:]
        output = cv2.rotate(output, cv2.ROTATE_90_CLOCKWISE)#[:,:350]
        # warped_img = cv2.warpPerspective(img, M, (IMAGE_H, IMAGE_W)) # Image warping
        return output

    def merge(self, head, tail, left, right, car):
        if head.ndim == 3:
            # horizontal = np.concatenate([np.zeros((640,179,3)),left,np.zeros((640,236,3)),right,np.zeros((640,179,3))],1)
            side_H, side_W, _ = left.shape
            # head_H, head_W, _ = head.shape
            total_width = self.car_width+side_W+side_W
            
            horizontal = np.concatenate([left,np.zeros((side_H,self.car_width,3)),right],1)
            horizontal = cv2.resize(horizontal, dsize=(horizontal.shape[1],800), interpolation = cv2.INTER_LINEAR)
            tail = cv2.resize(tail, dsize=(total_width,600), interpolation = cv2.INTER_LINEAR)
            head = cv2.resize(head, dsize=(total_width,600), interpolation = cv2.INTER_LINEAR)
            self.head_H, _, _ = head.shape
            #head = head/255#np.concatenate([np.zeros((400,(800-500)//2,3)),head/255,np.zeros((400,(800-500)//2,3))],1)
            
            # head_empty = np.zeros((140,head.shape[1],3)).astype(np.uint8)
            # tail_empty = np.zeros((140,tail.shape[1],3)).astype(np.uint8)
            # bev = np.concatenate([head,head_empty,horizontal,tail_empty,tail],0)
            bev_wo_car = np.concatenate([head, horizontal, tail], 0)
            bev = bev_wo_car.copy()
            bev[head.shape[0]-25:head.shape[0]+self.car_height-25,side_W:side_W+self.car_width,:] = self.car_final
            bev = (bev).astype(np.uint8)
            bev_wo_car = (bev_wo_car).astype(np.uint8)
            
        if head.ndim == 2:
            # horizontal = np.concatenate([np.zeros((640,179,3)),left,np.zeros((640,236,3)),right,np.zeros((640,179,3))],1)
            side_H, side_W = left.shape
            # head_H, head_W, _ = head.shape
            total_width = self.car_width+side_W+side_W
            
            horizontal = np.concatenate([left,np.zeros((side_H,self.car_width)),right],1)
            horizontal = cv2.resize(horizontal, dsize=(horizontal.shape[1],800), interpolation = cv2.INTER_LINEAR)
            tail = cv2.resize(tail, dsize=(total_width,600), interpolation = cv2.INTER_LINEAR)
            head = cv2.resize(head, dsize=(total_width,600), interpolation = cv2.INTER_LINEAR)
            self.head_H, _ = head.shape
            #head = head/255#np.concatenate([np.zeros((400,(800-500)//2,3)),head/255,np.zeros((400,(800-500)//2,3))],1)
            
            # head_empty = np.zeros((140,head.shape[1],3)).astype(np.uint8)
            # tail_empty = np.zeros((140,tail.shape[1],3)).astype(np.uint8)
            # bev = np.concatenate([head,head_empty,horizontal,tail_empty,tail],0)
            bev_wo_car = np.concatenate([head, horizontal, tail], 0)
            bev = bev_wo_car.copy()
            # bev[head.shape[0]-25:head.shape[0]+self.car_height-25,side_W:side_W+self.car_width] = self.car_final
            bev = (bev).astype(np.uint8)
            bev_wo_car = (bev_wo_car).astype(np.uint8)
        # tt = np.zeros((3300, 1600))
        #bev = Image.fromarray(bev)
        return bev, bev_wo_car


    """
    lane detection
    """
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

        if color == 'green':
            mask = cv2.inRange(hsv, (25, 60, 50), (86, 255, 255))
        elif color == 'red':
            mask = cv2.inRange(hsv, (115, 100, 50), (130, 255, 255))
        elif color == 'blue':
            mask = cv2.inRange(hsv, (10, 150, 50), (30, 255, 255))
        elif color == 'yellow':
            # mask = cv2.inRange(hsv, (40, 80, 105), (160, 255, 255))
            mask = cv2.inRange(hsv, (40, 60, 80), (160, 255, 255))
        
        imask = mask > 0
        temp = np.zeros_like(hsv, np.uint8)
        temp[imask] = 255
        output = self.image_clean(temp[:,:,0])

        return output

    """
    def fit_poly(self, img_shape, leftx, lefty, rightx, righty):
        ### TO-DO: Fit a second order polynomial to each with np.polyfit() ###
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        # Generate x and y values for plotting
        ploty = np.linspace(0, img_shape[0]-1, img_shape[0])
        ### TO-DO: Calc both polynomials using ploty, left_fit and right_fit ###
        left_fitx = left_fit[0] * (ploty ** 2) + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * (ploty ** 2) + right_fit[1] * ploty + right_fit[2]
        
        return left_fitx, right_fitx, ploty, left_fit, right_fit

    def search_around_poly(self, binary_warped, left_fit, right_fit):
        # HYPERPARAMETER
        # Choose the width of the margin around the previous polynomial to search
        margin = 10

        # Grab activated pixels
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        ### TO-DO: Set the area of search based on activated x-values ###
        ### within the +/- margin of our polynomial function ###
        ### Hint: consider the window areas for the similarly named variables ###
        ### in the previous quiz, but change the windows to our new search area ###
        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin))
                        & (nonzerox < (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin)))
        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) 
                        & (nonzerox < (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))
        

        # Again, extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Fit new polynomials
        left_fitx, right_fitx, ploty, left_fit_refined, right_fit_refined = fit_poly(binary_warped.shape, leftx, lefty, rightx, righty)
        
        ## Visualization ##
        # Create an image to draw on and an image to show the selection window
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
        window_img = np.zeros_like(out_img)
        # Color in left and right line pixels
        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [0, 0, 255]
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
        result = cv2.fillPoly(window_img.copy(), np.int_([left_line_pts]), (0,255,0))
        nonzero_final_left = result.nonzero()
        ly = np.array(nonzero_final_left[0])
        lx = np.array(nonzero_final_left[1])
        result = cv2.fillPoly(window_img.copy(), np.int_([right_line_pts]), (0,255,0))
        nonzero_final_right = result.nonzero()
        ry = np.array(nonzero_final_right[0])
        rx = np.array(nonzero_final_right[1])
        # result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
        # cv2.imwrite('xxx.png', result)
        # Plot the polynomial lines onto the image
        # plt.plot(left_fitx, ploty, color='yellow')
        # plt.plot(right_fitx, ploty, color='yellow')
        ## End visualization steps ##
        
        # return result, left_fit_refined, right_fit_refined, left_fitx, right_fitx, ploty
        # ly = nonzeroy[left_lane_inds]
        # lx = nonzerox[left_lane_inds]
        # ry = nonzeroy[right_lane_inds]
        # rx = nonzerox[right_lane_inds]
        return result, ly, lx, ry, rx

    def lane_detect(self, img):
        img_ori = img
        img_hsv = self.hsv(img, color = 'yellow')

        img_clean = self.clean(img_hsv)
        # out_img = img_clean
        # print(img_clean.shape)
        # out_img = img_clean
        out_img = cv2.cvtColor(img_clean, cv2.COLOR_GRAY2RGB)
        print(out_img.shape)
        # print("xxxxxxdddfdfdfdfd    x")

        # HYPERPARAMETERS
        # Choose the number of sliding windows
        nwindows = 10

        # Set the width of the windows +/- margin
        margin = 50

        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Set height of windows - based on nwindows above and image shape
        window_height = np.int64(img_clean.shape[0]//nwindows)

        # Take a histogram of the bottom half of the image
        histogram = np.sum(img_clean[img_clean.shape[0]-window_height:img_clean.shape[0],:], axis=0)

        midpoint = np.int64(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nonzero = img_clean.nonzero()
        # print(nonzero[1])
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # print(nonzerox)
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
            win_y_low = img_clean.shape[0] - (window+1)*window_height
            win_y_high = img_clean.shape[0] - window*window_height
            ### TO-DO: Find the four below boundaries of the window ###
            win_xleft_low = leftx_current - margin  # Update this
            win_xleft_high = leftx_current + margin  # Update this
            win_xright_low = rightx_current - margin  # Update this
            win_xright_high = rightx_current + margin  # Update this

            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low, win_y_low),(win_xleft_high, win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 
            # plt.imshow(xx)
            ### TO-DO: Identify the nonzero pixels in x and y within the window ###
            good_left_inds = ((nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high) & (nonzeroy >= win_y_low) & (nonzeroy < win_y_high)).nonzero()[0]
            good_right_inds = ((nonzerox >= win_xright_low) & (nonzerox < win_xright_high) & (nonzeroy >= win_y_low) & (nonzeroy < win_y_high)).nonzero()[0]

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            print(left_lane_inds, "aa")

            right_lane_inds.append(good_right_inds)

            ### TO-DO: If you found > minpix pixels, recenter next window ###
            ### (`right` or `leftx_current`) on their peak histogram ###
            if(len(good_left_inds) > minpix):
                leftx_current = np.int64(np.mean(nonzerox[good_left_inds]))
            if(len(good_right_inds) > minpix):
                rightx_current = np.int64(np.mean(nonzerox[good_right_inds]))

            # Concatenate the arrays of indices (previously was a list of lists of pixels)
        
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        # print(left_lane_inds)
        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        ### TO-DO: Fit a second order polynomial to each using `np.polyfit` ###
        # print('dd')
        # print(leftx)

        left_fit = np.polyfit(lefty, leftx, 2)
        # print('dad')

        right_fit = np.polyfit(righty, rightx, 2)
        # print('dsd')


        ploty = np.linspace(0, img_clean.shape[0]-1, img_clean.shape[0] )
        # plot
        # print(ploty)
        # try:
        #     left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        #     right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        # except TypeError:
        #     # Avoids an error if `left` and `right_fit` are still none or incorrect
        #     print('The function failed to fit a line!')
        #     left_fitx = 1*ploty**2 + 1*ploty
        #     right_fitx = 1*ploty**2 + 1*ploty

        ## Visualization ##
        # Colors in the left and right lane regions
        lane, ly, lx, ry, rx = self.search_around_poly(img_clean, left_fit, right_fit)
        img_ori[ly, lx] = (0, 0, 255)
        img_ori[ry, rx] = (0, 0, 255)
        output = img_ori

        return output, lane
    """

    def process(self):
        # 최초 시작
        if self.is_front and self.is_left and self.is_right and self.is_back:
            if self.is_first:
                
                '''
                front -> right -> rear -> left
                '''
                img1 = self.cur_img_front
                img4 = self.cur_img_left
                img2 = self.cur_img_right
                img3 = self.cur_img_back
	
                head = self.front(img1)
                tail = self.rear(img3)
                left = self.side_left(img4)
                right = self.side_right(img2)
                
                # merge
                _, self.old_frame = self.merge(head, tail, left, right, self.car)

                # self.old_frame = cv.cvtColor(self.old_frame, cv.COLOR_BGR2RGB)
                self.old_gray = cv.cvtColor(self.old_frame, cv.COLOR_BGR2GRAY)
                
                self.old_frame_head = cv.cvtColor(head[:,250:-250], cv.COLOR_BGR2RGB)
                self.old_gray_head = cv.cvtColor(self.old_frame_head, cv.COLOR_BGR2GRAY)
                print(self.old_gray_head.shape)

                
                head0 = self.front(np.ones(img1.shape))
                tail0 = self.rear(np.ones(img3.shape))
                left0 = self.side_left(np.ones(img4.shape))
                right0 = self.side_right(np.ones(img2.shape))

                _, mask = self.merge(head0,tail0,left0,right0,np.ones(self.car.shape))
                mask_inverse = 1-mask
                self.mask = mask
                self.mask_inverse = mask_inverse
                
                self.is_first = False
                cv2.imshow('surround view', cv2.resize(self.mask, dsize=(300,500)))
                # cv2.imshow('check', cv2.resize(self.mask, dsize=(300,500)))
                # cv2.imshow('lane', cv2.resize(self.mask, dsize=(300,500)))
                cv2.imshow('final', cv2.resize(self.mask, dsize=(600,500)))
    

                while True: # 무한 루프
                    keycode = cv2.waitKey() # 키보드 입력 반환 값 저장
                    if keycode == ord('i') or keycode == ord('I'): # i 또는 I
                        break
                
            else:
                img1 = self.cur_img_front
                img4 = self.cur_img_left
                img2 = self.cur_img_right
                img3 = self.cur_img_back  
		
                # image wrapping
                head = self.front(img1)
                tail = self.rear(img3)
                left = self.side_left(img4)
                right = self.side_right(img2)


                # lane detection
                head_lane = self.hsv(head)
                tail_lane = self.hsv(tail)
                left_lane = self.hsv(left)
                right_lane = self.hsv(right)
                
                # image mrege
                _, frame = self.merge(head, tail, left, right, self.car)
                _, frame_with_lane = self.merge(head_lane, tail_lane, left_lane, right_lane, self.car)
             
                try:
                    step = -10
                    old_frame = frame
                    self.old_frame[step*-1:,:,:] = self.old_frame[:step,:,:] # 이전 farame의 처음 10개를 마지막 10개로 변경
                    #import pdb;pdb.set_trace()
                    
                    # out_frame = frame
                    frame = frame*self.mask+self.old_frame*self.mask_inverse
                    out_frame = frame


                    out_frame[out_frame < 0] = 0
                    frame_with_lane[frame_with_lane < 0] = 0
                                            
                    self.prev_surround_view = out_frame
                    out_frame[self.head_H-25:self.head_H+self.car_height-25,left.shape[1]:left.shape[1]+self.car_width,:] = self.car_final
                    out_frame = out_frame[:,70:-70,:]
                    self.is_front = False
                    self.is_left = False
                    self.is_right = False
                    self.is_back = False

                    # image show
                    cv2.imshow('surround view', cv2.resize(out_frame, dsize=(300,500)))
                    # cv2.imshow('check', cv2.resize(old_frame, dsize=(300,500)))
                    # cv2.imshow('lane', cv2.resize(frame_with_lane, dsize=(300,500)))
                    t = cv2.resize(old_frame, dsize=(300,500))
                    # print(t.shape)
                    tt = cv2.resize(frame_with_lane, dsize=(300,500))
                    tt = cv2.cvtColor(tt, cv2.COLOR_GRAY2BGR)
                    # print(tt.shape)

                    ttt = np.concatenate([t, tt], 1)
                    cv2.imshow('final',ttt)

                    # image save                
                    cv2.imwrite('/home/juntae/catkin_ws/src/caffeine/src/surround view.png', out_frame)
                    cv2.imwrite('/home/juntae/catkin_ws/src/caffeine/src/check.png', old_frame)
                    cv2.imwrite('/home/juntae/catkin_ws/src/caffeine/src/lane.png', frame_with_lane)
                    cv2.imwrite('/home/juntae/catkin_ws/src/caffeine/src/final.png', ttt)
                    if self.iter == 5:
                        cv2.imwrite('/home/juntae/catkin_ws/src/caffeine/src/seq/front/' + str(self.save) + '.png', self.cur_img_front)
                        cv2.imwrite('/home/juntae/catkin_ws/src/caffeine/src/seq/right/' + str(self.save) + '.png', self.cur_img_right)
                        cv2.imwrite('/home/juntae/catkin_ws/src/caffeine/src/seq/left/' + str(self.save) + '.png', self.cur_img_left)
                        self.save += 1
                        self.iter = 0
                    self.iter += 1


                    cv2.waitKey(1)
                    print('nice\n')
                
                except:
                    print('error')
		
        else:
            print("NOT ALL IMAGES RECIEVED YET.") 
            
            
if __name__ == '__main__':
    rospy.init_node('surround_view_node')
    r = rospy.Rate(10)
    sv = SurroundView()

    while not rospy.is_shutdown():
        sv.process()
        r.sleep()
    
    rospy.spin()