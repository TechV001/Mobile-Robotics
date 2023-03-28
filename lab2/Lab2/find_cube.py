import cv2
import numpy as np
import time

def filter_image(img, hsv_lower, hsv_upper):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    
    #output = cv2.bitwise_and(img,img, mask= mask)
    
    img2 = cv2.medianBlur(mask, 7)
    
    kernel = np.ones((3, 3), np.uint8)
    output = cv2.dilate(img2, kernel, iterations=4)
    
    return output

    ###############################################################################
    ### You might need to change the parameter values to get better results
    ###############################################################################
def detect_blob(mask):
    img = cv2.medianBlur(mask, 9)
   # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 256;
    #filter by color (on binary)
    params.filterByColor = True
    params.blobColor = 255  # this looks at binary image 0 for looking for dark areas
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 2
    params.maxArea = 20000
    # Filter by Circularity
    params.filterByCircularity = False
    # Filter by Convexity
    params.filterByConvexity = False
    # Filter by Inertia
    params.filterByInertia = False
    detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs.
    keypoints = detector.detect(img)
    return keypoints

def find_cube(img, hsv_lower, hsv_upper):
    """Find the cube in an image.
        Arguments:
        img -- the image
        hsv_lower -- the h, s, and v lower bounds
        hsv_upper -- the h, s, and v upper bounds
        Returns [x, y, radius] of the target blob, and [0,0,0] or None if no blob is found.
    """
    mask = filter_image(img, hsv_lower, hsv_upper)
    keypoints = detect_blob(mask)

    if not keypoints:
        return None
    ###############################################################################
    # Todo: Sort the keypoints in a certain way if multiple key points get returned
    ###############################################################################
    keypoints = sorted(keypoints, key=lambda x: x.size, reverse=True)
   
    # Return the coordinates of the largest blob
    x = int(keypoints[0].pt[0])
    y = int(keypoints[0].pt[1])
    radius = int(keypoints[0].size / 2)
   
    return [x, y, radius]
