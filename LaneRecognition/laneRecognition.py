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
while(True):
	ret, frame = camera.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (7, 7), 0)
	threshMean = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, 10)
	threshGaussian = cv2.adaptiveThreshold(blurred,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY,11,2)
	openingMean = cv2.morphologyEx(threshMean, cv.MORPH_OPEN, kernel)
	openingGaussian = cv2.morphologyEx(threshGaussian, cv.MORPH_OPEN, kernel)
	titles = ['Original Image', 'Gray',
            'GaussianBlur', 'Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding','Opening Mean','Opening Gaussian']
	images = [frame, gray, blurred, threshMean, threshGaussian, openingMean, openingGaussian]
	for i in range(7):
	    plt.subplot(2,2,i+1),plt.imshow(images[i],'gray')
	    plt.title(titles[i])
	    plt.xticks([]),plt.yticks([])
	cv2.imshow('JetRacer Camera', plt)
	if cv2.waitKey(1) & 0xFF == ord('q'):
    		break
camera.release()
cv2.destroyAllWindows()
