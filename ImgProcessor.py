
#
# Robot Wars II - ImgProcessor
# Author:  Rick Phillips
# Date:    2013-05-15
# This class handles the image processing tasks needed for the various competitions.  Some functions are simply wrappers
# around opencv calls; the goal here is to encapsulate all of the opencv functionality that we think we need.
#
# TBD:  Some sort of calibraiton routine for the color thresholding.  Becasue no matter how well your color values work
# when you're testing in your kitchen, the light at the competition will make colors look totally different.  Trust me.
#



import colorsys
import cv2
import math
import numpy as np
import time

THRESHOLD_TYPE_NORMAL     = 0
THRESHOLD_TYPE_INVERSE    = 1


class ImgProcessor():

    def __init__(self, frameWidth, frameHeight):
        '''Class Constructor'''

        self.contourList = []
        self.debug = True
        self.frameWidth = frameWidth
        self.frameHeight = frameHeight
        self.shrinkFactor = 0.25
        self.threshHsvLower = [0, 0, 0]
        self.threshHsvUpper = [0, 0, 0]
        self.threshGrayLvl = 127
        self.processColorImg = False
        self.rectRoi = (0,0,frameWidth,frameHeight)
        self.rotated = False
        self.threshType = cv2.THRESH_BINARY_INV

        # test purposes
        self.threshImg = None
        self.contourImg = None

    def isImgRotated(self):
        '''Return whether img is rotated 180 deg.'''
        return self.rotated

    def getRoiImg(self, img):
        '''Apply region of interest and return resulting section of image'''
        x1 = self.rectRoi[0]
        y1 = self.rectRoi[1]
        x2 = self.rectRoi[2] * self.shrinkFactor
        y2 = self.rectRoi[3] * self.shrinkFactor
        roiImg = img[y1:y2, x1:x2]
        return roiImg
        

    def getNumVertices(self, contour):
        '''Return the number of vertices that define the contour'''
        return len(contour)


    def getCircles(self, img, minRadius=10, maxRadius=50):
        '''Get list of circles in the image'''

        newImg = self.shrinkImg(img)
        
        circles = None
        circleList = []
        if img is not None:
        
            roiImg = self.getRoiImg(newImg)

            # Convert to grayscale and apply thresholding
            grayImg = cv2.cvtColor(roiImg, cv2.COLOR_BGR2GRAY)
            ret, threshImg = cv2.threshold(grayImg, self.threshGrayLvl, 255, self.threshType)
            if threshImg is not None:
                if self.debug:
                    self.threshImg = threshImg.copy()
                
                # Use Hough Transform to detect circles
                #param1 = grayscale or thresholded image
                #param2 = detection method
                #param3 = inverse ratio of accumulator resolution to image resolution. huh?  set to 1
                #param4 = min distance between centers
                #param5 = "param1" - Higher threshold of the two passed to Canny edge-detection
                #param6 = "param2" - Accumulator threshold for circle centers.  Smaller = more false alarms
                #param8 = min radius
                #param9 = max radius
                circleTupleList = []
                circles = cv2.HoughCircles(threshImg,
                                           cv2.cv.CV_HOUGH_GRADIENT,
                                           1,
                                           (minRadius * 2),
                                           param1=90,
                                           param2=10,
                                           minRadius=minRadius,
                                           maxRadius=maxRadius)
                
            # Convert returned data into a list of tuples: (x_center, y_center, radius)
            # (make it easier for the client to use the results)
            if circles is not None:
                for circle in circles[0]:
                    circleList.append((circle[0], circle[1], circle[2]))
                
        return circleList
        
    def getContourList(self, img):
        '''Get list of contours present in the image'''

        # shrink image to reduce processing time
        newImg = self.shrinkImg(img)

        # Apply region of interest
        roiImg = self.getRoiImg(newImg)

        # Apply thresholding to get a binary image
        threshImg = None
        if self.processColorImg:
            
            # Convert the colorspace of our frame from BGR to HSV
            hsvImg = cv2.cvtColor(roiImg, cv2.COLOR_BGR2HSV)            
            threshImg = cv2.inRange(hsvImg, self.threshHsvLwr, self.threshHsvUpr)

        else:
            # Make sure we're really dealing with a grayscale image
            grayImg = cv2.cvtColor(roiImg, cv2.COLOR_BGR2GRAY)
            
            # Peform binary threshold operation            
            ret, threshImg = cv2.threshold(grayImg, self.threshGrayLvl, 255, self.threshType)

        contours = None
        heirarchy = None
        if threshImg is not None:
            if self.debug:
                # For test scripts you can display threshImg in its own window
                self.threshImg = threshImg.copy()            
            contours, heirarchy = cv2.findContours(threshImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        return contours, heirarchy


    def setShrinkFactor(self, factor):
        '''Set amount by which to scale down the frame'''
        self.shrinkFactor = factor


    def shrinkImg(self, img):
        '''Reduce the size of the img'''
        self.frameWidth = int(img.shape[1] * self.shrinkFactor)
        self.frameHeight = int(img.shape[0] * self.shrinkFactor)
        newImg = cv2.resize(img, (self.frameWidth, self.frameHeight))

        return newImg


    def getFrameWidth(self):
        '''Return width of frame to be processed'''
        return self.frameWidth
        

    def getContourCenter(self, contour):
        '''Return the centroid (x,y) of a contour'''
        x = 0
        y = 0
        moments = cv2.moments(contour)
        if moments['m00'] != 0.0:
            cx = moments['m10'] / moments['m00']
            cy = moments['m01'] / moments['m00']
            centroid = (cx, cy)

	
            #print 'Raw Centroid: ' + str(centroid)

            # Centroid is in respect to ROI.  Get centroid
            # w.r.t. full frame.
            x = int(math.floor(centroid[0] + self.rectRoi[0]))
            y = int(math.floor(centroid[1] + self.rectRoi[1]))

            # If we have the camera upside down, get equivalent x and y
            if self.rotated:
                x = self.frameWidth - x
                y = self.frameHeight - y
            
        return (x, y)


    def getContourArea(self, contour):
        '''Return area of the contour in pixels'''
        area = 0
        if contour is not None:
            area = cv2.contourArea(contour)
        return area


    def getPercentageOfFrame(self, contour):
        '''Return percentage of frame that a contour occupies'''
        frameArea = self.frameHeight * self.frameWidth
        return (self.getContourArea(contour) / frameArea)


    def setThresholdType(self, ttype):
        '''Set thresholding type: normal or inverse'''
        if ttype == THRESHOLD_TYPE_NORMAL:
            self.threshType = cv2.THRESH_BINARY
            
        elif ttype == THRESHOLD_TYPE_INVERSE:
            self.threshType = cv2.THRESH_BINARY_INV


    def setRgbThreshold(self, rgbLower, rgbUpper):
        '''Set the threshold range for a color image'''

        # So far I haven't determined if it's easier to specify color range in RGB
        # or straight HSV.  If RGB is your thing then this will convert your range
        # to HSV colorspace
        
        self.processColor = True        
        self.threshHsvLower = colorsys.rgb_to_hsv(rgbLower)
        self.threshHsvUpper = colorsys.rgb_to_hsv(rgbUpper)


    def setGrayThreshold(self, grayLvl):
        '''Set the threshold value for a grayscal image'''
        self.processColor = False
        
        if grayLvl < 0:
            garyLvl = 0
        if grayLvl > 255:
            grayLvl = 255
            
        self.threshGrayLvl = grayLvl


    def setImgRotated(self, rotated):
        '''Set whether image is rotated 180 deg. or not'''
        self.rotated = rotated
                                    

    def setRoi(self, rectRoi):
        '''Set the rectangular Region Of Interest'''
        self.rectRoi = rectRoi


    def resetRoi(self):
        '''Reset the Region Of Interest to full frame'''
        self.rectRoi = (0,0,self.frameWidth,self.frameHeight)
