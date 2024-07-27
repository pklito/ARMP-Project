import cv2
import pyrealsense2 as rs
import numpy as np
import sys
import threading
import signal
import time

class FakeCameraStreamer:
    def __init__(self, path, depth_path=None, width = 640, height = 360, framerate = 60):
        self.WIDTH = width
        self.HEIGHT = height
        self.FRAMERATE = framerate

        self.cap = cv2.VideoCapture(path)
        self.depth_cap = None
        if depth_path is not None:
            self.depth_cap = cv2.VideoCapture(depth_path)

        # Create a lock to synchronize access to the frames
        self.lock = threading.Lock()
        self.running = True
        # Create a thread for collecting frames
        self.collect_thread = threading.Thread(target=self.collect_frames)
        self.collect_thread.start()

        # Initialize variables for storing the frames
        self.color_image = None
        self.depth_image = None
        self.depth_frame = None
        self.depth_colormap = None
        self.depth_intrinsics = None
        self.is_new = False

    def collect_frames(self):
        while True:
            start_time = time.time()
            if not self.running:
                return

            ret2 = None
            ret, frame = self.cap.read()
            if not ret:
                return
            frame = cv2.resize(frame, (self.WIDTH, self.HEIGHT))
            if self.depth_cap is not None:
                ret2, dframe = self.depth_cap.read()
                dframe = cv2.resize(dframe, (self.WIDTH, self.HEIGHT))


            with self.lock:
                self.color_image = frame
                if ret2 is not None:
                    self.depth_colormap = dframe
                self.is_new = True
            end_time = time.time()
            remainder = 1/self.FRAMERATE - (end_time - start_time)
            if  remainder > 0:
                time.sleep(remainder)

    def get_frames(self):
        with self.lock:
            color_image = self.color_image
            depth_colormap = self.depth_colormap
            was_new = self.is_new
            self.is_new = False

        return color_image, None, None, depth_colormap, None, was_new


    def stop(self):
        self.running = False
        self.collect_thread.join()

    def stream(self):
        try:
            while True:
                color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics, is_new_image = self.get_frames()
                if color_image is not None and depth_colormap is not None:
                    images = np.hstack((color_image, depth_colormap))
                    cv2.imshow('RealSense Color and Depth Stream', images)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = FakeCameraStreamer("C:/Users/paulo/Videos/Screen Recordings/Square path color only.mp4")
    while True:
        color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics, is_new_image = camera.get_frames()
        if color_image is not None:
            cv2.imshow('color', color_image)
            cv2.waitKey(1)
