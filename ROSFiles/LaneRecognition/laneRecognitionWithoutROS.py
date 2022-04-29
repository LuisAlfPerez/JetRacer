#!/usr/bin/env python3

from matplotlib import pyplot as plt
from setupFiles.lineFunction import LineFunction
#import rospy
#from std_msgs.msg import Int32
import numpy
import cv2
import os

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=10,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def lineDetection(image, y_begin, y_final, width, threshold, minLineLength, maxLineGap):
    #Threshold: The minimum number of intersections to "*detect*" a line
    section = numpy.array([
        [(0,y_begin),(0, y_final),(width,y_final),(width, y_begin)]
        ])
    maskSection = numpy.zeros_like(image)
    cv2.fillPoly(maskSection, numpy.int32([section]), 255)
    maskSection = cv2.bitwise_and(image, maskSection)
    linesDetected= cv2.HoughLinesP(maskSection, 1, numpy.pi/180, threshold, numpy.array([]), minLineLength=minLineLength, maxLineGap=maxLineGap)
    return linesDetected    

def simplifyLines(lines, x_begin, x_final, y_begin, y_final, tolerance):
    newLines = []
    #print(lines.shape[0])
    for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            if newLines is not None:
                found = False
                searchingFunction = LineFunction(x1, y1, x2, y2, y_begin, y_final)
                for savedLines in newLines:
                    if abs(searchingFunction.slope) > 1.5:
                        if abs(searchingFunction.calculateYValue(x_begin)-savedLines.calculateYValue(x_begin))<=tolerance and abs(searchingFunction.calculateYValue(x_final)-savedLines.calculateYValue(x_final))<=tolerance:
                            found = True
                            savedLines.checkMinAndMax(searchingFunction, y_begin, y_final)
                            #"""
                    elif abs(searchingFunction.slope) > 1:
                        if abs(searchingFunction.calculateYValue(x_begin)-savedLines.calculateYValue(x_begin))<=tolerance*2 and abs(searchingFunction.calculateYValue(x_final)-savedLines.calculateYValue(x_final))<=tolerance*2:
                            found = True
                            savedLines.checkMinAndMax(searchingFunction, y_begin, y_final)
                    elif abs(searchingFunction.slope) > 0.5:
                        if abs(searchingFunction.calculateXValue(y_begin)-savedLines.calculateXValue(y_begin))<=tolerance*2 and abs(searchingFunction.calculateXValue(y_final)-savedLines.calculateXValue(y_final))<=tolerance*2:
                            found = True
                            savedLines.checkMinAndMax(searchingFunction, y_begin, y_final)
                    #"""
                    else :
                        if abs(searchingFunction.calculateXValue(y_begin)-savedLines.calculateXValue(y_begin))<=tolerance*4 and abs(searchingFunction.calculateXValue(y_final)-savedLines.calculateXValue(y_final))<=tolerance*4:
                            found = True
                            savedLines.checkMinAndMax(searchingFunction, y_begin, y_final)
                if not found:
                    newLines.append(searchingFunction)
            else :
                new_line = LineFunction(x1, y1, x2, y2, y_begin, y_final)
                newLines.append(new_line)
    straightLines = []
    for line in newLines:
        #print()
        #print(line.slope)
        if abs(line.slope) > 0.25:
            #print("se guarda")
            straightLines.append(line)
    return straightLines

def printLines(image, lines, width, height):
    lineImage = numpy.zeros_like(image)
    if lines is not None:
        for line in lines:
            if line.slope > 0:
                x1, y1, x2, y2 = [line.x_min, line.y_min, line.x_max, line.y_max]
            else :
                x1, y1, x2, y2 = [line.x_min, line.y_max, line.x_max, line.y_min]
            cv2.line(lineImage, (x1, y1), (x2, y2), (255, 255, 255), 10)
    cv2.line(lineImage, (round(width/2), 0), (round(width/2), height), (127, 127, 127), 2)
    lines = cv2.addWeighted(image, 0.8, lineImage, 1,1)
    plt.imshow(lines)
    plt.show()
    return lines    

def fillRegion(image, lines, width, height):
    if lines is None:
        return
    mask = numpy.zeros_like(image)
    refx=round(width/2)
    x1=0
    x2=width
    for y in range(height):
        x1 = 0
        x2 = width
        for line in lines:
            if line.y_min <= y and line.y_max >= y:
                x = line.calculateXValue(y)
                #print("x =",x,"y =",y,"min =",line.y_min,"max =",line.y_max, "cond =",x<refx, x<x2)
                if x < refx:
                    if x > x1:
                        x1 = x
                else:
                    if x < x2:
                        x2 = x
                        #print(x2)
        if y < round(height/2):
            if x1!=0 and x2!=width:
                #print("x1 =", x1, "x2 =",x2, "y =",y)
                cv2.line(mask, (x1, y), (x2, y), (255, 255, 255), 1)
        else:
            cv2.line(mask, (x1, y), (x2, y), (255, 255, 255), 1)
    return mask

def distanceFromReference(lines, width, referenceValue):
    if lines is None:
        return
    refx=round(width/2)
    x1=0
    x2=width
    y=referenceValue
    for line in lines:
        if line.y_min <= y and line.y_max >= y:
            x = line.calculateXValue(y)
            # print(x)
            #print("x =",x,"y =",y,"min =",line.y_min,"max =",line.y_max, "cond =",x<refx, x<x2)
            if x < refx:
                if x > x1:
                    x1 = x
            else:
                if x < x2:
                    x2 = x
                    #print(x2)
    # print(x1)
    # print(x2)
    # print(width/2)
    print("Left: ", int(x1), " Right: ", int(x2), "Center", int(width/2), " Distance from reference: ", int((x2+x1)/2-round(width/2)))
    #publisher.publish(int(x2-x1-round(width/2)))
    return 

def region_of_interest(imageReceived):
    height = imageReceived.shape[0]
    width = imageReceived.shape[1]
    croppedImage = imageReceived[0:height-1, 0:width-130-1]
    width = croppedImage.shape[1]
    # fieldView = 60*2.5
    # view = numpy.array([
    #     [(0,200),(width/2-fieldView, 0),(width/2+fieldView, 0),(width,200),(width, height),(0, height)]
    #     ])
    # mask = numpy.zeros_like(croppedImage)
    # cv2.fillPoly(mask, numpy.int32([view]), 255)
    # croppedImage = cv2.bitwise_and(croppedImage, mask)

    y_begin = 150
    y_final = 500
    threshold = 300
    minLineLength = 35
    maxLineGap = 50
    tolerance = 50
    referenceYValue = 400

    lineImage = numpy.zeros_like(croppedImage)
    lines1 = lineDetection(croppedImage, y_begin, y_final, width, threshold, minLineLength, maxLineGap)
    if lines1 is not None: 
        lines1 = simplifyLines(lines1, 0, width, y_begin, y_final, tolerance)
    distanceFromReference(lines1, width, referenceYValue)
    refImage = fillRegion(printLines(croppedImage, lines1, width, height), lines1, width, height)
    plt.imshow(refImage)
    #plt.show() 

    
#publisher = rospy.Publisher('referenceDistance', Int32, queue_size=1)
#rospy.init_node('vision', anonymous=True)

#image = cv2.imread("01-24-2022_16-10-45.jpg")
#image = cv2.imread("recta2.jpg")
#image = cv2.imread("curva.jpg")

#Recta
image = cv2.imread("04-14-2022_12-33-02.jpg")
#Curva izquierda
#image = cv2.imread("04-23-2022_14-39-19.jpg")
#Curva derecha
#image = cv2.imread("04-23-2022_14-40-01.jpg")
#Recta visión digonal
#image = cv2.imread("04-23-2022_14-42-23.jpg")
#Curva izquierda en U
#image = cv2.imread("04-23-2022_14-43-01.jpg")
#image = cv2.imread("04-23-2022_14-43-35.jpg")
#image = cv2.imread("04-23-2022_14-43-54.jpg")
#image = cv2.imread("04-23-2022_14-44-11.jpg")
#S derecha
#image = cv2.imread("04-23-2022_14-44-37.jpg")
#image = cv2.imread("04-23-2022_14-44-57.jpg")
image = cv2.imread("04-28-2022_23-56-06.jpg")

frame = numpy.copy(image)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (9, 9), 0)
kernelErosion = numpy.ones((2,2),numpy.uint8)
kernelDilate = numpy.ones((15,15),numpy.uint8)
kernelOpening = numpy.ones((3,3),numpy.uint8)
        
#MEAN
threshMean = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 25, 8)
erosionMean = cv2.erode(threshMean,kernelErosion,iterations = 4)
dilateMean = cv2.dilate(erosionMean,kernelDilate,iterations = 1)
openingMean = cv2.morphologyEx(threshMean, cv2.MORPH_OPEN, kernelOpening)
region_of_interest(dilateMean)
"""
camera = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
if camera.isOpened():
    keyCode = 0
    while keyCode != 27:
        # Stop the program on the ESC key
        keyCode = cv2.waitKey(30) & 0xFF
        ret, frame = camera.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 0)
        kernelErosion = numpy.ones((2,2),numpy.uint8)
        kernelDilate = numpy.ones((15,15),numpy.uint8)
        kernelOpening = numpy.ones((3,3),numpy.uint8)
        
        #MEAN
        threshMean = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 25, 8)
        erosionMean = cv2.erode(threshMean,kernelErosion,iterations = 4)
        dilateMean = cv2.dilate(erosionMean,kernelDilate,iterations = 1)
        openingMean = cv2.morphologyEx(threshMean, cv2.MORPH_OPEN, kernelOpening)
        region_of_interest(dilateMean)
else:
    print("Camera was not opened")
    """
titles = ['Original Image', 'Gray',
        'GaussianBlur','Adaptive Mean Thresholding','Erosion Mean','Dilate Mean','Opening Mean']
images = [frame, gray, blurred, threshMean, erosionMean, dilateMean, openingMean]
plt.figure('JetRacer Camera')
for i in range(6):
    plt.subplot(2,3,i+1),plt.imshow(images[i],'gray')
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
#plt.show()

"""
#GAUSSIAN
threshGaussian = cv2.adaptiveThreshold(blurred,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,49,8)
erosionGaussian = cv2.erode(threshGaussian,kernelErosion,iterations = 3)
dilateGaussian = cv2.dilate(erosionGaussian,kernelDilate,iterations = 1)
openingGaussian = cv2.morphologyEx(threshGaussian, cv2.MORPH_OPEN, kernelOpening)
titles = ['Original Image', 'Gray',
        'GaussianBlur','Adaptive Gaussian Thresholding','Erosion Gaussian','Dilate Gaussian','Opening Gaussian']
images = [frame, gray, blurred, threshGaussian, erosionGaussian, dilateGaussian, openingGaussian]
plt.figure('JetRacer Camera')
for i in range(7):
    plt.subplot(3,3,i+1),plt.imshow(images[i],'gray')
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
#plt.show()

#CANNY
canny = cv2.Canny(blurred, 50, 75)
#plt.imshow(canny)
#plt.show() 
#region_of_interest(canny)
titles = ['Original Image', 'Gray',
        'GaussianBlur','Canny']
images = [frame, gray, blurred, canny]
plt.figure('JetRacer Camera')
for i in range(4):
    plt.subplot(2,2,i+1),plt.imshow(images[i],'gray')
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
#plt.show()
#cv2.imshow('JetRacer Camera', plt)
#camera.release()
"""
cv2.destroyAllWindows()
