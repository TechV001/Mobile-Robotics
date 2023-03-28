'''
COMP4500 - Mobile Robotics 1
Lab 1
University of Massachusetts Lowell

Fabian Barrios
'''

import cv2
import numpy as np
import os
import glob

#TODO: Modify these values for yellow color range.
#Add separate thresholds for detecting green.

lower_yellow = np.array([10, 175, 150])
upper_yellow = np.array([20, 230, 235])

lower_green = np.array([16, 50, 10])
upper_green = np.array([100, 200, 80])

img_path = './data/'
img_files = glob.glob(img_path + '*.jpg')


#TODO: Change this function so that it filters the image based
#      on color using the hsv range for each color.
def filter_image(img, hsv_lower, hsv_upper):
    # Modify mask
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    
    #output = cv2.bitwise_and(img,img, mask= mask)
    
    img2 = cv2.medianBlur(mask, 7)
    
    kernel = np.ones((3, 3), np.uint8)
    output = cv2.dilate(img2, kernel, iterations=4)
    
    return output

    
#TODO: Change the parameters to make blob detection more accurate.
#      Hint: You might need to set some parameters to specify features
#      such as color, size, and shape.
#      The features have to be selected based on the application. 
def detect_blob(mask, blob_color):
    
    params = cv2.SimpleBlobDetector_Params()
    
    params.filterByCircularity = False
    params.filterByInertia = False;
    params.filterByConvexity = False;
    
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 10000

    params.filterByColor = True;
    params.blobColor = blob_color;

    # builds a blob detector with the given parameters 
    detector = cv2.SimpleBlobDetector_create(params)

    # use the detector to detect blobs.
    keypoints = detector.detect(mask)

    return len(keypoints)

    
def count_cubes(img):
    
    mask_yellow = filter_image(img, lower_yellow, upper_yellow)
    num_yellow = detect_blob(mask_yellow, 255)
    
    mask_green = filter_image(img, lower_green, upper_green)
    cv2.normalize(mask_green, mask_green, 0, 255, cv2.NORM_MINMAX)
    num_green = detect_blob(mask_green, 255)
    
    cv2.imshow("yellow mask", mask_yellow)
    print("num of yellow cubes: %d", num_yellow)
    cv2.imshow("green mask", mask_green)
    print("num of green cubes: %d", num_green)
    #TODO: Modify to return number of detected cubes for both yellow and green (instead of 0)
    return (num_yellow, num_green)
    

if __name__=='__main__':
   i = 0
   print("Use 0 key to go to next image.")
   for img_file in img_files:
      
      img = cv2.imread(img_file, -1)
      i = i + 1
      print("Reading image %d", i)
      
      
      cv2.imshow("Cube Window", img)
      num_yellow, num_green = count_cubes(img)
      
      cv2.waitKey(0)
      
      cv2.destroyAllWindows()
      
         
      