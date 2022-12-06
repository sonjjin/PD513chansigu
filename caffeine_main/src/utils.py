import numpy as np
import cv2

def hsv_parking(img, color='yellow'):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    if color == 'green':
        mask = cv2.inRange(hsv, (30, 90, 80), (80, 255, 255))
        imask = mask > 0
        output = np.zeros_like(hsv, np.uint8)
        output[imask] = 255
        output = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
        output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))

        output = cv2.morphologyEx(output, cv2.MORPH_DILATE, kernel)
        # output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
        return output
    
    elif color == 'red':
        mask = cv2.inRange(hsv, (110, 100, 100), (150, 255, 255))
        imask = mask > 0
        output = np.zeros_like(hsv, np.uint8)
        output[imask] = 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        output = cv2.morphologyEx(output, cv2.MORPH_DILATE, kernel)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
        # output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
        # output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
        return output

    elif color == 'blue':
        mask = cv2.inRange(hsv, (0, 150, 100), (20, 255, 255))
        imask = mask > 0
        output = np.zeros_like(hsv, np.uint8)
        output[imask] = 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        output = cv2.morphologyEx(output, cv2.MORPH_DILATE, kernel)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
        # output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
        # output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
        return output

    elif color == 'yellow':
        mask = cv2.inRange(hsv, (80, 40, 145), (150, 255, 255))
        imask = mask > 0
        temp = np.zeros_like(hsv, np.uint8)
        temp[imask] = 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10,10))
        clean = cv2.morphologyEx(temp[:,:,0], cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
        output = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)
        return output

    elif color == 'purple':
        mask = cv2.inRange(hsv, (130, 170, 130), (180, 255, 150))
        imask = mask > 0
        output = np.zeros_like(hsv, np.uint8)
        output[imask] = 255
        # mask = cv2.inRange(hsv, (80, 100, 145), (150, 255, 255))
        return output
    
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