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

def distanceFromReference(lines, width, referenceValueCloser, referenceValueMiddle, referenceValueFurther):
    refx=round(width/2)
    
    x_left_closer=0
    x_right_closer=width
    y_closer=referenceValueCloser

    x_left_middle=0
    x_right_middle=width
    y_middle=referenceValueMiddle

    x_left_further=0
    x_right_further=width
    y_further=referenceValueFurther

    if lines:
        for line in lines:
            if line.y_min <= y_closer and line.y_max >= y_further:
                x = line.calculateXValue(y_closer)
                if x < refx:
                    if x > x_left_closer:
                        x_left_closer = x
                else:
                    if x < x_right_closer:
                        x_right_closer = x

                x = line.calculateXValue(y_middle)
                if x < refx:
                    if x > x_left_middle:
                        x_left_middle = x
                else:
                    if x < x_right_middle:
                        x_right_middle = x

                x = line.calculateXValue(y_further)
                if x < refx:
                    if x > x_left_further:
                        x_left_further = x
                else:
                    if x < x_right_further:
                        x_right_further = x

    k_closer = 0
    k_middle = 1           #***************
    k_further = 1          #***************

    x_left = (k_closer*x_left_closer + k_middle*x_left_middle + k_further*x_left_further)/(k_closer+k_middle+k_further)
    x_right = (k_closer*x_right_closer + k_middle*x_right_middle + k_further*x_right_further)/(k_closer+k_middle+k_further)                
    
    distance_from_reference = int((x_right+x_left)/2-round(width/2))

    print("Distance from reference: ", distance_from_reference)
    return 
def region_of_interest(imageReceived):
    height = imageReceived.shape[0]
    width = imageReceived.shape[1]
    croppedImage=imageReceived
    # fieldView = 60*2.5
    # view = numpy.array([
    #     [(0,200),(width/2-fieldView, 0),(width/2+fieldView, 0),(width,200),(width, height),(0, height)]
    #     ])
    # mask = numpy.zeros_like(croppedImage)
    # cv2.fillPoly(mask, numpy.int32([view]), 255)
    # croppedImage = cv2.bitwise_and(croppedImage, mask)

    y_begin = 0
    y_final = height
    threshold = 150
    minLineLength = 35
    maxLineGap = 30
    tolerance = 30
    referenceYValueCloser = int(height*3/4) #400
    referenceYValueMiddle = int(height/2) #400
    referenceYValueFurther = int(height/4) #400

    lineImage = numpy.zeros_like(croppedImage)
    lines1 = lineDetection(croppedImage, y_begin, y_final, width, threshold, minLineLength, maxLineGap)
    # print(lines1)
    if lines1 is not None: 
        lines1 = simplifyLines(lines1, 0, width, y_begin, y_final, tolerance)
    distanceFromReference(lines1, width, referenceYValueCloser, referenceYValueMiddle, referenceYValueFurther)
    refImage = fillRegion(printLines(croppedImage, lines1, width, height), lines1, width, height)
    # plt.imshow(refImage)
    # plt.show() 

    
#publisher = rospy.Publisher('referenceDistance', Int32, queue_size=1)
#rospy.init_node('vision', anonymous=True)
#Recta
# image = cv2.imread("05-12-2022_21-45-17.jpg")
# image = cv2.imread("05-12-2022_21-49-43.jpg")
# image = cv2.imread("05-12-2022_21-50-13.jpg")

#Curva izquierda
# image = cv2.imread("05-12-2022_21-47-07.jpg")
# image = cv2.imread("05-12-2022_21-48-15.jpg")
# image = cv2.imread("05-12-2022_21-51-17.jpg")

#Curva derecha
# image = cv2.imread("05-12-2022_21-46-13.jpg")
# image = cv2.imread("05-12-2022_21-49-17.jpg")

image = cv2.imread("06-02-2022_22-16-26.764596.jpg")



height = image.shape[0]
width = image.shape[1]
reduced_height_up = int(4*height/9)
reduced_height_bottom = int(7*height/9)

image = image[reduced_height_up:reduced_height_bottom-1, 0:width-130-1]

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
# titles = ['Original Image', 'Gray',
#         'GaussianBlur','Adaptive Mean Thresholding','Erosion Mean','Dilate Mean','Opening Mean']
# images = [frame, gray, blurred, threshMean, erosionMean, dilateMean, openingMean]
# plt.figure('JetRacer Camera')
# for i in range(6):
#     plt.subplot(2,3,i+1),plt.imshow(images[i],'gray')
#     plt.title(titles[i])
#     plt.xticks([]),plt.yticks([])
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
