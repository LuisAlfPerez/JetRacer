from matplotlib import pyplot as plt
import numpy
import cv2
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
def gallery(array, ncols=3):
    nindex, height, width, intensity = array.shape
    nrows = nindex//ncols
    assert nindex == nrows*ncols
    # want result.shape = (height*nrows, width*ncols, intensity)
    result = (array.reshape(nrows, ncols, height, width, intensity)
              .swapaxes(1,2)
              .reshape(height*nrows, width*ncols, intensity))
    return result

def region_of_interest(imageReceived):
    height = image.shape[0]
    width = image.shape[1]
    croppedImage = imageReceived[0:height-1, 0:width-130-1]
    width = croppedImage.shape[1]

    #Main Lane
    fieldView = 60*2.5
    mainLane = numpy.array([
        [(0,200),(width/2-fieldView, 0),(width/2+fieldView, 0),(width,200),(width, height),(0, height)]
        ])
    mask = numpy.zeros_like(croppedImage)
    cv2.fillPoly(mask, numpy.int32([mainLane]), 255)
    maskedImageMainLane = cv2.bitwise_and(croppedImage, mask)
    #plt.imshow(maskedImageMainLane)
    #plt.show()
    linesDetectedMainLane = cv2.HoughLinesP(maskedImageMainLane, 1, numpy.pi/180, 300, numpy.array([]), minLineLength=200, maxLineGap=300)
    #print(linesDetected)
    lineImage = numpy.zeros_like(croppedImage)
    
    if linesDetectedMainLane is not None:
        for line in linesDetectedMainLane:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(lineImage, (x1, y1), (x2, y2), (255, 255,255), 2)
    #plt.imshow(frame)
    #plt.show()
    lines = cv2.addWeighted(croppedImage, 0.8, lineImage, 1,1)
    plt.imshow(lines)
    plt.show()

    #20 cm 
    closerSection = numpy.array([
        [(0,200),(width/2-fieldView, 275),(width/2+fieldView, 275),(width,200),(width, height),(0, height)]
        ])
    cv2.fillPoly(mask, numpy.int32([closerSection]), 255)
    maskedImageCloserSection = cv2.bitwise_and(croppedImage, mask)
    #plt.imshow(maskedImageMainLane)
    #plt.show()
    linesDetectedCloserSection = cv2.HoughLinesP(maskedImageCloserSection, 1, numpy.pi/180, 300, numpy.array([]), minLineLength=200, maxLineGap=300)
    #print(linesDetected)    
    lineImage = numpy.zeros_like(croppedImage)
    if linesDetectedCloserSection is not None:
        for line in linesDetectedCloserSection:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(lineImage, (x1, y1), (x2, y2), (255, 255,255), 2)
    #plt.imshow(frame)
    #plt.show()
    lines = cv2.addWeighted(croppedImage, 0.8, lineImage, 1,1)
    plt.imshow(lines)
    plt.show()


#camera = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
image = cv2.imread("01-24-2022_16-10-45.jpg")
frame = numpy.copy(image)
#ret, frame = camera.read()
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (9, 9), 0)
kernelErosion = numpy.ones((2,2),numpy.uint8)
kernelDilate = numpy.ones((15,15),numpy.uint8)
kernelOpening = numpy.ones((3,3),numpy.uint8)
#region_of_interest(frame)


#MEAN
threshMean = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 25, 8)
erosionMean = cv2.erode(threshMean,kernelErosion,iterations = 4)
dilateMean = cv2.dilate(erosionMean,kernelDilate,iterations = 1)
openingMean = cv2.morphologyEx(threshMean, cv2.MORPH_OPEN, kernelOpening)
titles = ['Original Image', 'Gray',
        'GaussianBlur','Adaptive Mean Thresholding','Erosion Mean','Dilate Mean','Opening Mean']
images = [frame, gray, blurred, threshMean, erosionMean, dilateMean, openingMean]
region_of_interest(dilateMean)
plt.figure('JetRacer Camera')
for i in range(7):
    plt.subplot(3,3,i+1),plt.imshow(images[i],'gray')
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
#plt.show()
#plt.show()

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
cv2.destroyAllWindows()
