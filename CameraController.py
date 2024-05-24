import cv2
import numpy as np
import os
from PIL import Image


def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit


def detect_object(frame):
    
    # blur the frame more
    frame = cv2.GaussianBlur(frame, (17, 17), 0)

    # dilute the frame
    kernel = np.ones((5, 5), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)

    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # lowerLimit, upperLimit = get_limits(color=orange)
    lowerLimit, upperLimit = (176, 154, 146), (179,  255, 255)
    lowerLimit2, upperLimit2 = (0, 154, 146), (9,  255, 255)
    
    mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
    mask2 = cv2.inRange(hsvImage, lowerLimit2,upperLimit2)
    mask = cv2.bitwise_or(mask, mask2)
    mask_ = Image.fromarray(mask)
    
    bbox = mask_.getbbox()
    
    return bbox


class CameraWrapper:
    def __init__(self):
        self.cap = cv2.VideoCapture('http://192.168.0.22:5000/video_feed')

    def is_visible(self, show=False, i=0):
        # Capture frame-by-frame
        ret, frame = self.cap.read()

        # If frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            return None
        bbox = detect_object(frame)

        if show:
            # Display the resulting frame
            if(bbox is not None):
                x1, y1, x2, y2 = bbox
                frame = cv2.rectangle(frame, (x1,y1),(x2,y2), (0,255,0), 5)
            cv2.imshow('Camera Stream', frame)

            # Break the loop on 'q' key press
            if cv2.waitKey(1) == ord('q'):
                return

        return bbox is not None

    def show_camera(self):
        i = 0
        while True:
            self.is_visible(show=True, i=i)
            i += 1

    def real_time(self):
        
        # clear vid folder
        if not os.path.exists('vid'):
            os.makedirs('vid')
        else:
            for file in os.listdir('vid'):
                os.remove(f'vid/{file}')

        i = 0
        while True:
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            
            # If frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                return None
            position = detect_object(frame)
            
            # Display the resulting frame
            if(position is not None):
                x1,y1,x2,y2 = position
                ball_center = (int((x1+x2)/2), int((y1+y2)/2))
                print(ball_center)
                frame = cv2.circle(frame.copy(), center=ball_center, radius=3, color=(0, 0, 0), thickness=-1)
            cv2.imshow('Camera Stream', frame)
            i += 1
            
            # Break the loop on 'q' key press
            if cv2.waitKey(1) == ord('q'):
                return


if __name__ == '__main__':
    camera = CameraWrapper()
    camera.real_time()
    # camera.show_camera()
