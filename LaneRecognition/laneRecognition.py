import nanocamera
import cv2
camera = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
#frame = camera.read()
while(True):
	ret, frame = camera.read()
	cv2.imshow('frame', frame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
    		break
camera.release()
cv2.destrolAllWindows()
