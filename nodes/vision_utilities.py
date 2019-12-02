
'''
'''

import cv2
import numpy as np
import urllib

'''
https://github.com/alexleavitt/uscplayspokemon/blob/master/tommycam.py
'''
def get_video_capture_frame(video_capture_url_jpg_str):
    img_request = urllib.request.urlopen(video_capture_url_jpg_str)
    img_arr = np.asarray(bytearray(img_request.read()), dtype=np.uint8)   
    original_frame = cv2.imdecode(img_arr,-1)
    return original_frame

'''
https://docs.opencv.org/master/d7/d4d/tutorial_py_thresholding.html
'''
def threshold_frame(img_frame, min_threshold):
    ret, thresholded_frame = cv2.threshold(img_frame, min_threshold, 255, cv2.THRESH_BINARY)
    return thresholded_frame


'''
https://docs.opencv.org/master/d4/d13/tutorial_py_filtering.html
https://docs.opencv.org/master/d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed
'''
def remove_noise(img_frame):
    result = img_frame
    result = cv2.bilateralFilter(result,16,32,32) 
    
    kernel = np.ones((2, 2),np.uint8)
    result = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel) 
    
    kernel = np.ones((9, 9),np.uint8)
    result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel)
   
    result = cv2.bilateralFilter(result,3,16,16)
    return result

'''
https://docs.opencv.org/master/df/d9d/tutorial_py_colorspaces.html
https://stackoverflow.com/questions/56905592/automatic-contrast-and-brightness-adjustment-of-a-color-photo-of-a-sheet-of-pape????
'''
def blue_color_mask(img_frame):
    result = img_frame   
    
    result = cv2.convertScaleAbs(result, alpha=1.95, beta=0)
    
    result = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([70,70,80])
    upper_blue = np.array([140,255,255])
    result = cv2.inRange(result, lower_blue, upper_blue)
    return result

def filter_blue_bricks(img_frame):
    result = img_frame
    # result = remove_noise(result)
    result = blue_color_mask(result)
    result = fint_brick_center(result) 
    return result

def filter_red_bricks(img_frame):
    result = img_frame
    # result = filter_excess_red_frame(img_frame)
    result = threshold_frame(result, 200)
    #result = remove_noise(result)
    result = fint_brick_center(result)    
    return result

def fint_brick_center(img_frame):
    result = img_frame
    contours, hierarchy = cv2.findContours(result.astype('uint8'), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for brick in contours:
        areal = cv2.contourArea(brick)
        if(areal > 1500):
            M = cv2.moments(brick)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            result = cv2.circle(result,(cx, cy),10,(0,255,0))
        elif(areal > 600):
            M = cv2.moments(brick)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            result = cv2.circle(result,(cx, cy),5,(0,255,0))
    return result
    
while True:
    dlink_video_jpg_url = 'http://192.168.0.20/image.jpg'
    dlink_video_jpg_url2 = 'http://192.168.0.21/image.jpg'
    img = get_video_capture_frame(dlink_video_jpg_url)
    cropped_img = img[42:img.shape[0]-80, 0:img.shape[1]] # crop so it only shows table
    
    blue_bricks = filter_blue_bricks(cropped_img)
    cv2.imshow('blue', blue_bricks)       
    
    # red_bricks = filter_red_bricks(cropped_img)
    # cv2.imshow('red', red_bricks)
    
    #img2 = get_video_capture_frame(dlink_video_jpg_url2)
    #cv2.imshow('vid2', img2)
    if cv2.waitKey(1) != -1:
        break
    
