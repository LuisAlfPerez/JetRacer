import cv2
import time
import os
from datetime import datetime

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=60,
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

def show_camera():
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        date_time = now.strftime("%m-%d-%Y_%H-%M-%S")
        ret_val, img = cap.read()
        cv2.imwrite(os.getcwd()+date_time, img)
        time.sleep(1)
        date_time = now.strftime("%m-%d-%Y_%H-%M-%S")
        ret_val, img = cap.read()
        cv2.imwrite(os.getcwd()+date_time, img)
        time.sleep(1)
        date_time = now.strftime("%m-%d-%Y_%H-%M-%S")
        ret_val, img = cap.read()
        cv2.imwrite(os.getcwd()+date_time, img)
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")


if __name__ == "__main__":
    show_camera()