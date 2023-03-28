'''
COMP4500 - Mobile Robotics 1
Lab 1
University of Massachusetts Lowell
'''

import cv2
import sys
import numpy as np
from glob import glob

datapath = 'data/'
imgs = [f for f in glob(datapath+'*') if '.jpg' in f]

def null(x):
    pass

# Create a window
#cv2.namedWindow('Default Mask')
cv2.namedWindow('Yellow Mask')
cv2.namedWindow('Green Mask')

# create trackbars for color change
cv2.createTrackbar('HMinY','Yellow Mask',0,179,null)
cv2.createTrackbar('SMinY','Yellow Mask',0,255,null)
cv2.createTrackbar('VMinY','Yellow Mask',0,255,null)
cv2.createTrackbar('HMaxY','Yellow Mask',0,179,null)
cv2.createTrackbar('SMaxY','Yellow Mask',0,255,null)
cv2.createTrackbar('VMaxY','Yellow Mask',0,255,null)

cv2.createTrackbar('HMinG','Green Mask',0,179,null)
cv2.createTrackbar('SMinG','Green Mask',0,255,null)
cv2.createTrackbar('VMinG','Green Mask',0,255,null)
cv2.createTrackbar('HMaxG','Green Mask',0,179,null)
cv2.createTrackbar('SMaxG','Green Mask',0,255,null)
cv2.createTrackbar('VMaxG','Green Mask',0,255,null)

# Set default value for MAX HSV trackbars.
cv2.setTrackbarPos('HMaxY', 'Yellow Mask', 179)
cv2.setTrackbarPos('SMaxY', 'Yellow Mask', 255)
cv2.setTrackbarPos('VMaxY', 'Yellow Mask', 255)

cv2.setTrackbarPos('HMaxG', 'Green Mask', 179)
cv2.setTrackbarPos('SMaxG', 'Green Mask', 255)
cv2.setTrackbarPos('VMaxG', 'Green Mask', 255)

# Initialize to check if HSV min/max value changes
hMinY = sMinY = vMinY = hMaxY = sMaxY = vMaxY = 0
#phMinY = psMinY = pvMinY = phMaxY = psMaxY = pvMaxY = 0

hMinG = sMinG = vMinG = hMaxG = sMaxG = vMaxG = 0
#phMinG = psMinG = pvMinG = phMaxG = psMaxG = pvMaxG = 0


#TODO: Change this function so that it filters the image based
#      on color using the hsv range for each color.
def filter_image(img, hsv_lower, hsv_upper):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    output = cv2.bitwise_and(img, img, mask=mask)
    return output

    
#TODO: Change the parameters to make blob detection more accurate.
#      Hint: You might need to set some parameters to specify features
#      such as color, size, and shape.
#      The features have to be selected based on the application. 
def detect_blob(mask):
    img = cv2.medianBlur(mask, 5)

    # Set up the SimpleBlobdetector with default parameters with specific values.
    params =  cv2.SimpleBlobDetector_Params()

    params.filterByInertia = False;
    params.filterByConvexity = False;
    
    params.filterByArea = True
    params.minArea = 30
    params.maxArea = 200

    params.filterByColor = True;
    params.blobColor = 255;

    # builds a blob detector with the given parameters 
    detector = cv2.SimpleBlobDetector_create(params)

    # use the detector to detect blobs.
    keypoints = detector.detect(img)

    return len(keypoints)

    
def count_cubes(img):
    mask_yellow = filter_image(img, yellow_lower, yellow_upper)
    num_yellow = detect_blob(mask_yellow)

    mask_green = filter_image(img, green_lower, green_upper)
    num_green = detect_blob(mask_green)

    #TODO: Modify to return number of detected cubes for both yellow and green (instead of 0)
    return num_yellow, num_green

for f in imgs:
    i = int(f[-6:-4])
    img = cv2.imread(f,-1)
    
    while True:

        # get current positions of all trackbars
        hMinY = cv2.getTrackbarPos('HMinY','Yellow Mask')
        sMinY = cv2.getTrackbarPos('SMinY','Yellow Mask')
        vMinY = cv2.getTrackbarPos('VMinY','Yellow Mask')

        hMaxY = cv2.getTrackbarPos('HMaxY','Yellow Mask')
        sMaxY = cv2.getTrackbarPos('SMaxY','Yellow Mask')
        vMaxY = cv2.getTrackbarPos('VMaxY','Yellow Mask')

        hMinG = cv2.getTrackbarPos('HMinG','Green Mask')
        sMinG = cv2.getTrackbarPos('SMinG','Green Mask')
        vMinG = cv2.getTrackbarPos('VMinG','Green Mask')
        
        hMaxG = cv2.getTrackbarPos('HMaxG','Green Mask')
        sMaxG = cv2.getTrackbarPos('SMaxG','Green Mask')
        vMaxG = cv2.getTrackbarPos('VMaxG','Green Mask')

        # Set minimum and max HSV values to display
        yellow_lower = np.array([hMinY, sMinY, vMinY])
        yellow_upper = np.array([hMaxY, sMaxY, vMaxY])

        green_lower = np.array([hMinG, sMinG, vMinG])
        green_upper = np.array([hMaxG, sMaxG, vMaxG])

        # Create HSV Image and threshold into a range.
        hsvY = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        maskY = cv2.inRange(hsvY, yellow_lower, yellow_upper)
        yellow_mask = cv2.bitwise_and(img, img, mask=maskY)
        
        hsvG = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        maskG = cv2.inRange(hsvG, green_lower, green_upper)
        green_mask = cv2.bitwise_and(img, img, mask=maskG)

        # Print if there is a change in HSV value
        #if( (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
        #    print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
        #    phMin = hMin
        #    psMin = sMin
        #    pvMin = vMin
        #    phMax = hMax
        #    psMax = sMax
        #    pvMax = vMax

        # Display output image
        #cv2.imshow('Default Mask', img)

        cv2.imshow("Green Mask", green_mask)
        
        cv2.imshow("Yellow Mask", yellow_mask)

        # Wait longer to prevent freeze for videos.
        if cv2.waitKey(33) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
