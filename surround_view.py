#!/usr/bin/env python

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
        self.img_left_sub = rospy.Subscriber('/left_cam/image_raw', Image, self.img_left_callback)
        self.img_right_sub = rospy.Subscriber('/right_cam/image_raw', Image, self.img_right_callback)
        self.img_back_sub = rospy.Subscriber('/rear_cam/image_raw', Image, self.img_back_callback)
        
        # 초기화
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

        # 이부분이 먼지 모르겠음 ㅋㅋ
        # 앞뒤 카메라 사각형 4점 좌표
        # 
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

        
        self.contours = 0
        self.chk_contours = -99

	
        # 자동차 이미지 불러오는 것 인듯
        #self.car = cv2.imread('./car.jpg')
        self.car = cv2.imread('/home/ka/catkin_ws/src/budicar.png')
        self.car = cv2.resize(self.car, dsize=(480, 640),interpolation=cv2.INTER_LINEAR)
        self.final_car = cv2.resize(self.car, dsize=(716, 910),interpolation=cv2.INTER_LINEAR)

        self.feature_params = dict(maxCorners = 30, 
                                   qualityLevel = 0.001,
                                    minDistance = 7,
                                    blockSize = 7
                                    )

        self.lk_params = dict(winSize = (5,5),
		maxLevel = 2,
		criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
        #self.final_car = cv2.resize(self.car, dsize=(420, 700),interpolation=cv2.INTER_LINEAR)
	
    def hsv(self, img, color='green'):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # 휴리스틱으로 구한거 같은데? 오진다,
        if color == 'green':
            mask = cv2.inRange(hsv, (25, 60, 50), (86, 255, 255))
        elif color == 'red':
            mask = cv2.inRange(hsv, (115, 100, 50), (130, 255, 255))
        elif color == 'blue':
            mask = cv2.inRange(hsv, (10, 150, 50), (30, 255, 255))
        elif color == 'yellow':
            # mask = cv2.inRange(hsv, (80, 40, 145), (150, 255, 255))
            mask = cv2.inRange(hsv, (80, 100, 145), (150, 255, 255))
        print(mask.shape)
        imask = mask > 0
        output = np.zeros_like(hsv, np.uint8)
        output[imask] = 255

        return output[:,:,0]

    def detect_square(self, input):
        min_area = 740
        H, W = input.shape[:2]
        # image morpholgy로 라인만 찾는거 같음
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9)) # kernel 만들기
        clean = cv2.morphologyEx(input, cv2.MORPH_OPEN, kernel) # 이럴거면 왜 이미지 하나로 opening, closing하는지?
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
        clean = cv2.morphologyEx(input, cv2.MORPH_CLOSE, kernel)

        contours = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #RETR_EXTERNAL: 외각선만 찾기, CHAIN_APPROX_SIMPLE: 수직선, 수평선, 대각선에 대해 끝점만 저장
        contours = contours[0] if len(contours) == 2 else contours[1]

        square = None
        square_center = 0
        is_square = False

        for c in contours:
            rot_rect = cv2.minAreaRect(c) # contour를 둘러싸는 최소한 크기의 직사각형 만들기
            temp_area = rot_rect[1][0] * rot_rect[1][1]
            temp_square = cv2.boxPoints(rot_rect)
            temp_center = np.int0(np.mean(temp_square, axis=0))

            if temp_area >= min_area and temp_center[0] > square_center:
                square = np.int0(temp_square)
                square_center = temp_center[0]
                area = temp_area
                is_square = True

        return square, is_square

    # callback 함수
    
    def img_front_callback(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
        self.cur_img_front = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	# print(self.cur_img_front.dtype) 
        self.is_front = True

    def img_left_callback(self, data):
        if not self.is_left:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
            self.cur_img_left = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
	    #print(self.cur_img_left.dtype) 
            self.is_left = True
    
    def img_right_callback(self, data):
        if not self.is_right:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
            self.cur_img_right = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
	    #print(self.cur_img_right.dtype) 
            self.is_right = True
    
    def img_back_callback(self, data):
        if not self.is_back:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
            self.cur_img_back = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
	    #print(self.cur_img_back.dtype) 
            self.is_back = True


    # surround view에서 빈 부분 채우는 함수인듯
    def fill(self,old_frame,frame,dx,dy):
        finx = dx
        if finx == 0:
            finx = old_frame.shape[1]
        finy = dy
        if finy == 0:
            finy = old_frame.shape[0]
        frame[650:,550:]=old_frame[650+dy:finy,550+dx:finx]
        frame[650:,-dx:250]=old_frame[650+dy:finy,max(dx,0):250+dx]
    
        return frame

    # front ~ side_right bird eye view로 바꾸는거
    def front(self,img):
        IMAGE_H, IMAGE_W, _ = img.shape
    
	    #print(img.shape)
    	#img = np.concatenate([np.zeros((400,250,3)).astype(np.uint8),img,np.zeros((400,250,3)).astype(np.uint8)],1)

        src = self.forward_src#np.float32([[249, 399], [549, 399], [289, 0], [509, 0]])
        dst = self.forward_dst#np.float32([[279, 399], [519, 399], [0, 0], [799, 0]])
        #src = np.float32([[210,115], [210,180], [150,120], [150,175]])
        #dst = np.float32([[210,115], [210,180], [150,115], [150,180]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation
    
        IMAGE_H, IMAGE_W, _ = img.shape
    
        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H))#[:300] # Image warping
    
        return warped_img#cv2.resize(warped_img[200:,100:-100], dsize=(800, 400),interpolation=cv2.INTER_LINEAR)#warped_img

    
    def rear(self,img):
        IMAGE_H, IMAGE_W, _ = img.shape
    
    	#img = np.concatenate([np.zeros((400,250,3)).astype(np.uint8),img,np.zeros((400,250,3)).astype(np.uint8)],1)
        src = self.backward_src#np.float32([[249, 399], [549, 399], [289, 0], [509, 0]])
        dst = self.backward_dst#np.float32([[279, 399], [519, 399], [0, 0], [799, 0]])
    	#src = np.float32([[210,115], [210,180], [150,120], [150,175]])
        #dst = np.float32([[210,115], [210,180], [150,115], [150,180]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation
    
        IMAGE_H, IMAGE_W, _ = img.shape
    
        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H))#[:300] # Image warping
    
        return warped_img#cv2.resize(warped_img[200:,100:-100], dsize=(800, 400),interpolation=cv2.INTER_LINEAR)#warped_img
    
    def side_left(self,img):
        
        IMAGE_H, IMAGE_W, _ = img.shape
        #src = np.float32([[0, 299], [399, 299], [0, 0], [399, 0]])
        #dst = np.float32([[0, 299], [399, 299], [100, 0], [299, 0]])
        src = np.float32([[0, 299], [399, 299], [0, 0], [399, 0]])
        dst = np.float32([[30, 299], [369, 299], [0, 0], [399, 0]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation.mkv
        
        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
        warped_img = cv2.rotate(warped_img, cv2.ROTATE_90_COUNTERCLOCKWISE)#[:,:350]
        
        return warped_img
        
    def side_right(self,img):
        
        IMAGE_H, IMAGE_W, _ = img.shape
        
        #src = np.float32([[0, 299], [399, 299], [0, 0], [399, 0]])
        #dst = np.float32([[0, 299], [399, 299], [100, 0], [299, 0]])
        src = np.float32([[0, 299], [399, 299], [0, 0], [399, 0]])
        dst = np.float32([[30, 299], [369, 299], [0, 0], [399, 0]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation.mkv

        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
        warped_img = cv2.rotate(warped_img, cv2.ROTATE_90_CLOCKWISE)#[:,:350]
        
        return warped_img


    def merge(self,head,tail,left,right,car):
        horizontal = np.concatenate([np.zeros((640,179,3)),left,np.zeros((640,236,3)),right,np.zeros((640,179,3))],1)
    	#print(head.shape)
        #print(tail.shape)
        tail = cv2.resize(tail, dsize=(1554,1170),interpolation=cv2.INTER_LINEAR)
    
        #tail = tail/255#tail = tail/255
	
        head = cv2.resize(head, dsize=(1554,1170),interpolation=cv2.INTER_LINEAR)
        #head = head/255#np.concatenate([np.zeros((400,(800-500)//2,3)),head/255,np.zeros((400,(800-500)//2,3))],1)
        
        head_empty = np.zeros((150,head.shape[1],3)).astype(np.uint8)
        tail_empty = np.zeros((170,tail.shape[1],3)).astype(np.uint8)
        bev = np.concatenate([head,head_empty,horizontal/255,tail_empty,tail],0)
        bev = (bev*255).astype(np.uint8)
        #bev = Image.fromarray(bev)
    
        return bev

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
	
		
                '''
                print(img1)
                    cv2.imwrite('/home/ka/front.jpg',img1)
                    cv2.imwrite('/home/ka/right.jpg',img2)
                    cv2.imwrite('/home/ka/rear.jpg',img3)
                    cv2.imwrite('/home/ka/left.jpg',img4)
                '''               
                #img1 = cv2.resize(img1, dsize=(600, 800),interpolation=cv2.INTER_LINEAR)
                #img1 = img1[200:-200,150:-150]
                #img3 = cv2.resize(img3, dsize=(600, 800),interpolation=cv2.INTER_LINEAR)
                #img3 = img3[200:-200,150:-150]
                #cv2.imwrite('head.jpg',img1)
                #img1 = cv2.resize(img1, dsize=(300, 400),interpolation=cv2.INTER_LINEAR)
                #img3 = cv2.resize(img3, dsize=(300, 400),interpolation=cv2.INTER_LINEAR)
                #img2 = cv2.resize(img2, dsize=(400, 300),interpolation=cv2.INTER_LINEAR)
                #img4 = cv2.resize(img4, dsize=(400, 300),interpolation=cv2.INTER_LINEAR)

                head = 255*self.front(img1)

                tail = 255*cv2.flip(cv2.flip(self.rear(img3),0),1)
                left = cv2.flip(self.side_left(img4),0)
                right = cv2.flip(self.side_right(img2),0)

                # merge
                self.old_frame = self.merge(head,tail,left,right,self.car)

                self.sero = (self.old_frame.shape[0]-910)//2 # sero가 뭐지?
                old_frame_out = self.old_frame
                old_frame_out[self.sero:-self.sero,(240+179):-(240+179)] = self.final_car


                self.old_frame = cv.cvtColor(self.old_frame, cv.COLOR_BGR2RGB)
                self.old_gray = cv.cvtColor(self.old_frame, cv.COLOR_BGR2GRAY)
                
                self.old_frame_head = cv.cvtColor(head[:,250:-250], cv.COLOR_BGR2RGB)
                self.old_gray_head = cv.cvtColor(self.old_frame_head, cv.COLOR_BGR2GRAY)
                print(self.old_gray_head.shape)
                
                self.is_first = False


                head0 = self.front(np.ones(img1.shape)*255*255)
                tail0 = cv2.flip(self.rear(np.ones(img3.shape)*255*255),0)
                left0 = cv2.flip(self.side_left(np.ones(img4.shape)*255),0)
                right0 = cv2.flip(self.side_right(np.ones(img2.shape)*255),0)

                mask = self.merge(head0,tail0,left0,right0,np.ones(self.car.shape)*255)//255
                remask = 1-mask
                self.mask = mask
                self.remask = remask
                
                
            else:
                img1 = self.cur_img_front
                img4 = self.cur_img_left
                img2 = self.cur_img_right
                img3 = self.cur_img_back  

                #img1 = cv2.resize(img1, dsize=(600, 800),interpolation=cv2.INTER_LINEAR)
                #img1 = img1[200:-200,150:-150]
                #img3 = cv2.resize(img3, dsize=(600, 800),interpolation=cv2.INTER_LINEAR)
                #img3 = img3[200:-200,150:-150]
                #img1 = cv2.resize(img1, dsize=(300, 400),interpolation=cv2.INTER_LINEAR)
                #img3 = cv2.resize(img3, dsize=(300, 400),interpolation=cv2.INTER_LINEAR)

                # Lucas kanade 저 좌표들이 뜻 하는게 무엇인지 파악할 필요가 있음
                p0 = np.zeros((32,1,2)).astype(np.float32)
                p0[0] = [400,680]
                p0[1] = [600,680]
                p0[2] = [1000,680]
                p0[3] = [1200,680]
		
                for i in range(1,8):
                    p0[i*4+0] = p0[0]+[20*i,0]
                    p0[i*4+1] = p0[1]+[20*i,0]
                    p0[i*4+2] = p0[2]+[10*i,0]
                    p0[i*4+3] = p0[3]+[10*i,0]
		
                head = 255*(self.front(img1))

                #tail = cv2.flip(self.front(img3),0)
                tail = 255*(cv2.flip(cv2.flip(self.rear(img3),0),1))
                left = self.side_left(img4)#cv2.flip(side_left(img4),0)
                right = self.side_right(img2)#cv2.flip(side_right(img2),0)

                frame = self.merge(head,tail,left,right,self.car)
                
                frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
                frame_gray_head = cv.cvtColor(head, cv.COLOR_BGR2GRAY)
		
                # calculate optical flow

                #p0 = cv2.goodFeaturesToTrack(self.old_gray_head, mask = None, **self.feature_params)
                """
                window size (5,5)로 lucas-kanade algorithm사용
                지정한 점들이 어디로 움직였는지 보여줌
                """
                p1, st, err = cv.calcOpticalFlowPyrLK(self.old_gray, frame_gray, p0, None, **self.lk_params) 

                good_new = p1[st==1]
                good_old = p0[st==1]

                print((good_old-good_new).shape)
                try:
                    #dx,dy = np.median((good_old-good_new).squeeze()[:,0]),np.median((good_old-good_new).squeeze()[:,1])
                    dx,dy = np.median((good_old-good_new)[:,0]),np.median((good_old-good_new)[:,1])
                    dx,dy = int(round(dx)),int(round(dy))
                        #if dx**2 > 9 or dy**2 > 9:
                        #    frame = frame
                        #else:
                        #    frame = frame#fill(old_frame,frame,dx,dy)
                    # print(dy)
                    
                        #old_frame[1:] = old_frame[:-1]
                    '''
                    if dy < 0:                
                        self.old_frame[min(dy,-1)*-1:] = self.old_frame[:min(dy,-1)]
                        else:
                        self.old_frame[:max(dy,1)*-1] = self.old_frame[max(dy,1):]
                    '''
                    step = -10
                    self.old_frame[step*-1:] = self.old_frame[:step] # 이전 farame의 처음 10개를 마지막 10개로 변경
                    
                    #import pdb;pdb.set_trace()
                    frame = frame*self.mask+self.old_frame*self.remask

                
                    self.old_gray = frame_gray.copy()
                    self.old_frame = frame.copy()


                    self.old_frame_head = cv.cvtColor(head[:,250:-250], cv.COLOR_BGR2RGB)
                    self.old_gray_head = cv.cvtColor(self.old_frame_head, cv.COLOR_BGR2GRAY)
                
                        
                    out_frame = frame
		        
                    if self.chk_contours == -99:
                        self.contours = 1-(cv2.dilate(self.remask,None,iterations=10)-self.remask)
                                
                    out_frame = out_frame*self.contours
                    #out_frame = out_frame-(remask-cv2.erode(remask,None,iterations=10))*255
                    out_frame[out_frame<0]=0
                    #import pdb;pdb.set_trace()
                    #cv2.imwrite('test.png',out_fram	e)
                                
                    
                    input_yellow = self.hsv(out_frame, color='yellow')  
                    
                    square_yellow, is_square = self.detect_square(input_yellow)
                    
                    if is_square:
                        center_yellow = np.int0(np.mean(square_yellow, axis=0))
                    
                        cv2.drawContours(out_frame, [square_yellow], 0, (0,0,255), 3, cv2.LINE_AA)
                                
                        
                    self.prev_surround_view = out_frame

                    self.is_front = False
                    self.is_left = False
                    self.is_right = False
                    self.is_back = False

                    out_frame[self.sero:-self.sero,(240+179):-(240+179)] = self.final_car
            
                        
                    #cv2.imshow('surround view', out_frame)
                    #cv2.imshow('surround view',cv2.resize(self.remask*255, dsize=(300, 500),interpolation=cv2.INTER_LINEAR))
                    '''			
                    for ii in range(p0.shape[0]):			
                        out_frame = cv2.circle(out_frame, (p0[ii][0][0],p0[ii][0][1]), 10, (0,0,255), 20)
                            for ii in range(p1.shape[0]):			
                        out_frame = cv2.circle(out_frame, (p1[ii][0][0],p1[ii][0][1]), 10, (255,0,0), 20)
                            '''
                    cv2.imshow('surround view',cv2.resize(out_frame, dsize=(300, 500),interpolation=cv2.INTER_LINEAR))
                    cv2.waitKey(1)
                    #cv2.imwrite('/home/ka/tttt.png',out_frame)
                    #import pdb;pdb.set_trace()
                    print('nice\n')
                
                except:
                    kkkkk = 1               
		
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
