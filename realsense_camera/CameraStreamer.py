import cv2
import pyrealsense2 as rs
import numpy as np
from PIL import Image
import os
import threading
import signal
import sys
import cv2.aruco as aruco
from localization import *
def signal_handler(sig, frame, cam):
    print("Ctrl-C detected. Stopping camera stream and closing OpenCV windows...")
    cam.stop()
    cv2.destroyAllWindows()
    sys.exit(0)


# from robot_utils.kinematics import calculate_plate_center

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
#	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
#	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
#	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
#	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

ARUCO_POSITIONS = {
    500: "top_left",
    400: "top_right",
    600: "bottom_right",
    300: "bottom_left",
    200: "center"
}
def calculate_similarity(color1, color2):
    return np.sum(np.abs(color1 - color2))

# Function to perform object detection on a frame (eggroll)
def detect_object(frame):

    frame = cv2.GaussianBlur(frame, (17, 17), 0)
    kernel = np.ones((5, 5), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)

    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_limit1, upper_limit1 = (0, 154, 146), (9,  255, 255)
    lower_limit2, upper_limit2 = (176, 154, 146), (179,  255, 255)

    mask1 = cv2.inRange(hsv_image, lower_limit1, upper_limit1)
    mask2 = cv2.inRange(hsv_image, lower_limit2, upper_limit2)
    mask = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    bounding_boxes = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        bounding_boxes.append((x, y, x + w, y + h))

    return bounding_boxes

def detect_ball(frame):
    frame = cv2.GaussianBlur(frame, (17, 17), 0)
    kernel = np.ones((5, 5), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)

    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_limit1, upper_limit1 = (0, 154, 146), (9,  255, 255)
    lower_limit2, upper_limit2 = (176, 154, 146), (179,  255, 255)

    mask1 = cv2.inRange(hsv_image, lower_limit1, upper_limit1)
    mask2 = cv2.inRange(hsv_image, lower_limit2, upper_limit2)
    mask = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    bounding_circles = []
    for contour in contours:
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        bounding_circles.append((center, radius))
    return bounding_circles

def detect_plate(frame):

    frame = cv2.GaussianBlur(frame, (17, 17), 0)
    kernel = np.ones((5, 5), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)

    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # TODO: adjust the limits to accommodate the plate
    lower_limit1, upper_limit1 = (0,0,143), (179,53,210)
    # lower_limit2, upper_limit2 = (176, 154, 146), (179,  255, 255)

    mask = cv2.inRange(hsv_image, lower_limit1, upper_limit1)
    # mask2 = cv2.inRange(hsv_image, lower_limit2, upper_limit2)
    # mask = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        contour_area = [cv2.contourArea(contour) for contour in contours]
        max_contour_index = np.argmax(contour_area)
        x, y, w, h = cv2.boundingRect(contours[max_contour_index])
        bounding_box = (x, y, x + w, y + h)
        return [bounding_box]
    else:
        return []

class CameraStreamer:
    def __init__(self):
        self.WIDTH = 640
        self.HEIGHT = 360
        # Initialize RealSense camera pipeline
        # self.cap = cv2.VideoCapture(2) # Intel's Realsense Camera is on my pc
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 30)
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

    def collect_frames(self):
        while True:
            frames = self.pipeline.wait_for_frames()
            with self.lock:
                self.depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                if not self.depth_frame or not color_frame:
                    continue

                self.depth_image = np.asanyarray(self.depth_frame.get_data())
                self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
                self.color_image = np.asanyarray(color_frame.get_data())
                self.depth_intrinsics = self.depth_frame.profile.as_video_stream_profile().intrinsics

    def get_frames(self):
        with self.lock:
            color_image = (self.color_image).copy() if self.color_image is not None else None
            depth_image = (self.depth_image).copy() if self.depth_image is not None else None
            depth_frame = (self.depth_frame)
            depth_colormap = (self.depth_colormap)
            depth_intrinsics = (self.depth_intrinsics)

        return color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics

    def detect_arucos(self, color_image):
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        arucoParams = cv2.aruco.DetectorParameters()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(color_image, arucoDict, parameters=arucoParams)
        if ids is not None:
            color_image_with_markers = cv2.aruco.drawDetectedMarkers(color_image.copy(), corners, ids)

            centers = []
            for id, corner in zip(ids, corners):
                print("id: " + str(id))
                # Corner order: [top-left, top-right, bottom-right, bottom-left]
                c = corner[0]
                center = (c[:, 0].mean(), c[:, 1].mean())
                centers.append(center)
                cv2.circle(color_image_with_markers, (int(center[0]), int(center[1])), 3 ,(0,255,0),2)
                if ARUCO_POSITIONS[id[0]] == 'center':
                    centers = [center]
                    break
            avg_center = np.mean(centers, axis=0)
            cv2.circle(color_image_with_markers, (int(avg_center[0]), int(avg_center[1])), 3 ,(255,0,0),2)
            cv2.imwrite("aruco_detected.jpg", color_image_with_markers)
            # cv2.imshow("aruco_detected.jpg", color_image_with_markers)
            return avg_center, color_image_with_markers
        else:
            print("No arucos detected")
            color_image_with_markers = color_image.copy()
        return None, color_image_with_markers

    def localization_detection(self,):
        try:
            while True:
                color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics = camera.get_frames()
                if color_image is None or depth_image is None:
                    continue

                if color_image.size == 0 or depth_image.size == 0:
                    print("Can't receive frame (stream end?).")
                    continue

                positions = detect_ball(color_image)
                if len(positions) == 0:
                    print('No object detected!')
                    continue

                ball_center, radius = positions[0]

                ids, corners = get_aruco_corners(color_image)
                wcpoints = (-1,-1,-1)
                if ids is not None:
                    object_pts = np.array([a for i in ids for a in get_object([i[0]])], dtype=np.float32)
                    pixel_pts = np.array([c for aruco in corners for c in aruco[0]], dtype=np.float32)
                    if(len(object_pts) != len(pixel_pts)):
                        print("Error, sizes", len(object_pts),len(pixel_pts))
                    else:
                        if pixel_pts.ndim == 3 and pixel_pts.shape[1] == 1 and pixel_pts.shape[2] == 4:
                            pixel_pts = pixel_pts[:, 0, :]

                        if object_pts.size == 0 or pixel_pts.size == 0:
                            return None

                        ret, rvec, tvec = cv2.solvePnP(object_pts, pixel_pts, CAMERA_MATRIX, CAMERA_DIST_COEFF)

                        if ret:
                            wcpoint = getPixelOnPlane((ball_center[0], ball_center[1]),rvec,tvec)
                            print([round(a,3) for a in wcpoint])

                            cv2.drawFrameAxes(color_image, CAMERA_MATRIX, CAMERA_DIST_COEFF, rvec, tvec, 0.026, 2)

                color = (0, 255, 0)  # Green for ball
                cv2.circle((color_image), ball_center, radius, color, 2) # the enclosing circle
                cv2.circle((color_image), ball_center, 2 ,color,2) # a dot in the middle of the circle
                cv2.putText((color_image), f'Ball Center: ({ball_center[0]}, {ball_center[1]}),', (ball_center[0], ball_center[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                images = np.hstack((color_image, depth_colormap))
                cv2.imshow('RealSense Stream', images)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.stop()
            cv2.destroyAllWindows()

    def stop(self):
        self.pipeline.stop()
        self.collect_thread.join()

    def stream(self):
        try:
            while True:
                color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics = self.get_frames()
                if color_image is not None and depth_colormap is not None:
                    images = np.hstack((color_image, depth_colormap))
                    cv2.imshow('RealSense Color and Depth Stream', images)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.stop()
            cv2.destroyAllWindows()

    def stream_frames(self, color_image, depth_colormap):
        if color_image is not None and depth_colormap is not None:
            images = np.hstack((color_image, depth_colormap))
            cv2.imshow('RealSense Color and Depth Stream', images)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
                cv2.destroyAllWindows()

    def get_world_position_from_camera(self, pixel_x, pixel_y, depth_frame_input = None):
        df = depth_frame_input
        if df is None:
            df = self.depth_frame
        if not df:
            return None

        depth = df.get_distance(pixel_x, pixel_y)
        depth_intrinsics = df.profile.as_video_stream_profile().intrinsics

        # Convert pixel coordinates to world coordinates (from the Camera's Perspective!)
        ball_position = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [pixel_x, pixel_y], depth)
        return np.array(ball_position)


    def run_object_detection(self):
        try:
            while True:
                color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics = self.get_frames()

                if self.color_image is None or self.depth_image is None:
                    continue

                object_bounding_boxes = detect_object(color_image)
                plate_bounding_boxes = [] # detect_plate(color_image)

                # Combine bounding boxes for both ball and plate
                all_bounding_boxes = object_bounding_boxes + plate_bounding_boxes

                for bbox in all_bounding_boxes:
                    x1, y1, x2, y2 = bbox
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    depth_value = self.depth_frame.get_distance(center_x, center_y)
                    distance_meters = depth_value * 1000  # Convert to millimeters
                    camera_world_coords = self.get_world_position_from_camera(center_x, center_y)

                    if bbox in object_bounding_boxes:
                        color = (0, 255, 0)  # Green for ball
                        object_type = "Ball"
                    else:
                        color = (0, 0, 255)  # Red for plate
                        object_type = "Plate"

                    cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(color_image, f'{object_type} Distance: {distance_meters:.2f} mm,', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(color_image, f'Center: ({center_x}, {center_y}),', (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(color_image, f'World Coordinates: {camera_world_coords}', (x1, y1 - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                images = np.hstack((color_image, depth_colormap))
                cv2.imshow('RealSense Color and Depth Stream', images)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.stop()
            cv2.destroyAllWindows()



if __name__ == '__main__':
    camera = CameraStreamer()
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, camera)) # may not work properly
    # camera.run_object_detection()
    # camera.stream()
    camera.localization_detection()
