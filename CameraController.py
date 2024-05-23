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
    orig = frame.copy()
    red = [0, 0, 255]  # red in BGR colorspace

    # blur the frame more
    frame = cv2.GaussianBlur(frame, (17, 17), 0)

    # dilute the frame
    kernel = np.ones((5, 5), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)

    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lowerLimit, upperLimit = get_limits(color=red)

    mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

    mask_ = Image.fromarray(mask)

    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox

        orig = cv2.rectangle(orig, (x1, y1), (x2, y2), (0, 255, 0), 5)

    return orig, (bbox is not None)


class CameraWrapper:
    def __init__(self):
        self.cap = cv2.VideoCapture('http://192.168.0.3:5000/video_feed')

    def is_visible(self, show=True, i=0):
        # Capture frame-by-frame
        ret, frame = self.cap.read()

        # If frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            return None
        frame, is_detected = detect_object(frame)

        if show:
            # cv2.imwrite(f'vid/frame_{i}.png', frame)
            # i += 1
            # Display the resulting frame
            cv2.imshow('Camera Stream', frame)

            # Break the loop on 'q' key press
            if cv2.waitKey(1) == ord('q'):
                return

        return is_detected

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

            # Display the resulting frame
            cv2.imshow('Camera Stream', frame)
            cv2.imwrite(f'vid/frame_{i}.png', frame)
            i += 1
            # Break the loop on 'q' key press
            if cv2.waitKey(1) == ord('q'):
                return


if __name__ == '__main__':
    camera = CameraWrapper()
    # camera.show_camera()
    camera.real_time()