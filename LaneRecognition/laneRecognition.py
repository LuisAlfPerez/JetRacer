import nanocamera
import cv2
camera = nanocamera.Camera(flip=0, width=1280, height=800, fps=30)
#frame = camera.read()
while(True):
	ret, frame = camera.read()
	cv2.imshow('frame', frame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
    		break
camera.release()
cv2.destrolAllWindows()
