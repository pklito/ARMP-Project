import cv2
import pyrealsense2 as rs
import numpy as np
import sys
import threading
import signal

def signal_handler(sig, frame, cam):
    #print("Ctrl-C detected. Stopping camera stream and closing OpenCV windows...")
    cam.stop()
    cv2.destroyAllWindows()
    sys.exit(0)


class CameraStreamer:
    def __init__(self, width = 640, height = 360, no_depth=False):
        self.WIDTH = width
        self.HEIGHT = height
        # Initialize RealSense camera pipeline
        # self.cap = cv2.VideoCapture(2) # Intel's Realsense Camera is on my pc
        self.pipeline = rs.pipeline()
        config = rs.config()
        self.no_depth = no_depth
        if no_depth:
            config.disable_stream(rs.stream.depth)
        else:
            config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 0)
        self.pipeline.start(config)

        # Create a lock to synchronize access to the frames
        self.lock = threading.Lock()

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
            frames = self.pipeline.wait_for_frames()
            with self.lock:
                if self.no_depth:
                    self.depth_frame = None
                    self.depth_image = None
                    self.depth_colormap = None
                    self.depth_intrinsics = None
                else:
                    self.depth_frame = frames.get_depth_frame()
                    self.depth_image = np.asanyarray(self.depth_frame.get_data())
                    self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
                    self.depth_intrinsics = self.depth_frame.profile.as_video_stream_profile().intrinsics

                color_frame = frames.get_color_frame()

                if (not self.depth_frame and not self.no_depth) or (not color_frame):
                    continue

                self.color_image = np.asanyarray(color_frame.get_data())
                self.is_new = True

    def get_frames(self):
        with self.lock:
            color_image = self.color_image
            depth_image = self.depth_image
            depth_frame = self.depth_frame
            depth_colormap = self.depth_colormap
            depth_intrinsics = self.depth_intrinsics
            was_new = self.is_new
            self.is_new = False

        return color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics, was_new


    def stop(self):
        self.pipeline.stop()
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
