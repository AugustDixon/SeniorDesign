from tower import *


import sys
import os
import shutil
import numpy as np
import cv2 as cv
import time as tiempo
import datetime


def main():

    # Setup LEDBlobDetector parameters
    params = cv.SimpleBlobDetector_Params()
    # Change thresholds
    # Image will be thresholded beforehand, meaning only one thresholding step is necessary
    params.minThreshold = 0
    params.maxThreshold = 256

    # Filter by Area
    params.filterByArea = True
    params.minArea = 7
    params.maxArea = 40

    # Filter by Circularity
    params.filterByCircularity = False
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = .5
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = .1

    ver = (cv.__version__).split('.')
    if int(ver[0]) < 3 :
        LEDBlobDetector = cv.SimpleBlobDetector(params)
    else : 
        LEDBlobDetector = cv.SimpleBlobDetector_create(params)
    
    
    # Filter by Area
    params.filterByArea = True
    params.minArea = 300
    params.maxArea = 1600
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = .70
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = .4
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = .05

    ver = (cv.__version__).split('.')
    if int(ver[0]) < 3 :
        ObstacleBlobDetector = cv.SimpleBlobDetector(params)
    else : 
        ObstacleBlobDetector = cv.SimpleBlobDetector_create(params)
        
    # Filter by Area
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 350

    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = .1
    params.maxCircularity = .70
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = .2
    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = .2

    ver = (cv.__version__).split('.')
    if int(ver[0]) < 3 :
        BlockBlobDetector = cv.SimpleBlobDetector(params)
    else : 
        BlockBlobDetector = cv.SimpleBlobDetector_create(params)
        
    params.minArea = 700
    params.maxArea = 4200
    
    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = .3
    params.maxCircularity = .70
    
    ver = (cv.__version__).split('.')
    if int(ver[0]) < 3 :
        BlockSurrBlobDetector = cv.SimpleBlobDetector(params)
    else : 
        BlockSurrBlobDetector = cv.SimpleBlobDetector_create(params)
    
    
    LowerLED = np.array([0, 0, 245])
    UpperLED = np.array([180, 51, 255])
    
    LowerMothershipLED = np.array([80, 102, 178])
    UpperMothershipLED = np.array([90, 204, 255])
    
    LowerBlockObstacle = np.array([0, 0, 165])
    UpperBlockObstacle = np.array([180, 51, 255])
    
    LowerBlockSurr = np.array([10, 90, 140])
    UpperBlockSurr = np.array([20, 165, 192])
    
    
    frameNum = 1
    path = '/home/pi/.virtualenvs/tower/lib/python3.5/site-packages/tower/data/'
    while True:
        image = cv.imread(path + ('frame%d.png' % frameNum))
        
        if (image is None) or (image.size == 0):
            break
        
        hsvImage = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        
        LEDmask = cv.inRange(hsvImage, LowerLED, UpperLED)
        inv_LEDmask = cv.bitwise_not(LEDmask)
        
        MothershipMask = cv.inRange(hsvImage, LowerMothershipLED, UpperMothershipLED)
        inv_MothershipMask = cv.bitwise_not(MothershipMask)
        
        BlockObstacleMask = cv.inRange(hsvImage, LowerBlockObstacle, UpperBlockObstacle)
        inv_BlockObstacleMask = cv.bitwise_not(BlockObstacleMask)
        
        BlockSurrMask = cv.inRange(hsvImage, LowerBlockSurr, UpperBlockSurr)
        
        
        ret = cv.imwrite(path + ("frame%dLEDmask.png" % frameNum), inv_LEDmask)
        ret = cv.imwrite(path + ("frame%dMothershipLEDMask.png" % frameNum), inv_MothershipMask)
        ret = cv.imwrite(path + ("frame%dBlockObstacleMask.png" % frameNum), inv_BlockObstacleMask)
        ret = cv.imwrite(path + ("frame%dBlockSurrMask.png" % frameNum), BlockSurrMask)
        
        
        gray = np.float32(BlockSurrMask)
        dst = cv.cornerHarris(gray,5,3,0.04)

        #result is dilated for marking the corners, not important
        dst = cv.dilate(dst,None)

        # Threshold for an optimal value, it may vary depending on the image.
        hsvImage[dst>0.2*dst.max()]=[0,0,255]
        ret = cv.imwrite(path + ("frame%dCorners.png" % frameNum), hsvImage)
        
        keypoints = LEDBlobDetector.detect(inv_LEDmask)
        im_with_keypoints = image.copy()
        print("LED Keypoints: %d" % len(keypoints))
        for marker in keypoints:
            im_with_keypoints = cv.drawMarker(im_with_keypoints, tuple(int(i) for i in marker.pt), color=(0, 255, 0))
        ret = cv.imwrite(path + ("frame%dLEDKeypoints.png" % frameNum), im_with_keypoints)
        
        keypoints = LEDBlobDetector.detect(inv_MothershipMask)
        im_with_keypoints = image.copy()
        print("Mothership LED Keypoints: %d" % len(keypoints))
        for marker in keypoints:
            im_with_keypoints = cv.drawMarker(im_with_keypoints, tuple(int(i) for i in marker.pt), color=(0, 255, 0))
        ret = cv.imwrite(path + ("frame%dMothershipLEDKeypoints.png" % frameNum), im_with_keypoints)
        
        keypoints = BlockBlobDetector.detect(BlockObstacleMask)
        im_with_keypoints = image.copy()
        print("Block Keypoints: %d" % len(keypoints))
        for marker in keypoints:
            im_with_keypoints = cv.drawMarker(im_with_keypoints, tuple(int(i) for i in marker.pt), color=(0, 255, 0))
        ret = cv.imwrite(path + ("frame%dBlockKeypoints.png" % frameNum), im_with_keypoints)
        
        keypoints = BlockSurrBlobDetector.detect(BlockSurrMask)
        im_with_keypoints = image.copy()
        print("Block Surr Keypoints: %d" % len(keypoints))
        for marker in keypoints:
            im_with_keypoints = cv.drawMarker(im_with_keypoints, tuple(int(i) for i in marker.pt), color=(0, 255, 0))
        ret = cv.imwrite(path + ("frame%dBlockSurrKeypoints.png" % frameNum), im_with_keypoints)
        
        keypoints = ObstacleBlobDetector.detect(inv_BlockObstacleMask)
        im_with_keypoints = image.copy()
        print("Obstacle Keypoints: %d" % len(keypoints))
        for marker in keypoints:
            im_with_keypoints = cv.drawMarker(im_with_keypoints, tuple(int(i) for i in marker.pt), color=(0, 255, 0))
        ret = cv.imwrite(path + ("frame%dObstacleKeypoints.png" % frameNum), im_with_keypoints)
        
        
        
        print("Frame %d finished" % frameNum)
        frameNum += 1
    #END OF while
    print("Script Complete")

#END OF main()



if __name__ == "__main__":
    main()
