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
camera = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
#frame = camera.read()
if (True):
	ret, frame = camera.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blurredMean = cv2.GaussianBlur(gray, (9, 9), 0)
	blurredGaussian = cv2.GaussianBlur(gray, (9, 9), 0)
	threshMean = cv2.adaptiveThreshold(blurredMean, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 25, 8)
	threshGaussian = cv2.adaptiveThreshold(blurredGaussian,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,49,8)
	kernelErosion = numpy.ones((2,2),numpy.uint8)
	erosionMean = cv2.erode(threshMean,kernelErosion,iterations = 4)
	erosionGaussian = cv2.erode(threshGaussian,kernelErosion,iterations = 3)
	kernelDilate = numpy.ones((15,15),numpy.uint8)
	dilateMean = cv2.dilate(erosionMean,kernelDilate,iterations = 1)
	dilateGaussian = cv2.dilate(erosionGaussian,kernelDilate,iterations = 1)
	kernelOpening = numpy.ones((3,3),numpy.uint8)
	openingMean = cv2.morphologyEx(threshMean, cv2.MORPH_OPEN, kernelOpening)
	openingGaussian = cv2.morphologyEx(threshGaussian, cv2.MORPH_OPEN, kernelOpening)
	titles = ['Original Image', 'Gray',
            'GaussianBlur', 'GaussianBlur', 'Adaptive Mean Thresholding','Erosion Mean','Dilate Mean','Opening Mean', 'Adaptive Gaussian Thresholding','Erosion Gaussian','Dilate Gaussian','Opening Gaussian']
	images = [frame, gray, blurredMean, blurredGaussian, threshMean, erosionMean, dilateMean, openingMean, threshGaussian, erosionGaussian, dilateGaussian, openingGaussian]
	plt.figure('JetRacer Camera')
	for i in range(12):
	    plt.subplot(3,4,i+1),plt.imshow(images[i],'gray')
	    plt.title(titles[i])
	    plt.xticks([]),plt.yticks([])
	plt.show()
	#cv2.imshow('JetRacer Camera', plt)
camera.release()
cv2.destroyAllWindows()
