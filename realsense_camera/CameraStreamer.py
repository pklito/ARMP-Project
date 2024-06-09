import cv2
from networkx import center
import pyrealsense2 as rs
import numpy as np
from PIL import Image
import os


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
        # Initialize RealSense camera pipeline
        # self.cap = cv2.VideoCapture(2) # Intel's Realsense Camera is on my pc
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.color_image, self.depth_image, self.depth_frame, self.depth_colormap, self.depth_intrinsics = self.get_frames()

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None, None, None

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        color_image = np.asanyarray(color_frame.get_data())
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        return color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics

    def stop(self):
        self.pipeline.stop()

    def stream(self):
        try:
            while True:
                self.color_image, self.depth_image, self.depth_frame, self.depth_colormap, self.depth_intrinsics = self.get_frames()
                if self.color_image is not None and self.depth_colormap is not None:
                    images = np.hstack((self.color_image, self.depth_colormap))
                    cv2.imshow('RealSense Color and Depth Stream', images)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.stop()
            cv2.destroyAllWindows()

    def get_world_position_from_camera(self, pixel_x=320, pixel_y=240):
        if not self.depth_frame:
            return None

        depth = self.depth_frame.get_distance(pixel_x, pixel_y)
        depth_intrinsics = self.depth_frame.profile.as_video_stream_profile().intrinsics

        # Convert pixel coordinates to world coordinates (from the Camera's Perspective!)
        ball_position = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [pixel_x, pixel_y], depth)
        return np.array(ball_position)


    def run_object_detection(self):
        try:
            if not os.path.exists('vid'):
                os.makedirs('vid')
            else:
                for file in os.listdir('vid'):
                    os.remove(f'vid/{file}')
            while True:
                self.color_image, self.depth_image, self.depth_frame, self.depth_colormap, self.depth_intrinsics = self.get_frames()

                if self.color_image is None or self.depth_image is None:
                    continue

                object_bounding_boxes = detect_object(self.color_image)
                plate_bounding_boxes = [] # detect_plate(color_image)  # Add plate detection

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

                    cv2.rectangle(self.color_image, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(self.color_image, f'{object_type} Distance: {distance_meters:.2f} mm,', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(self.color_image, f'Center: ({center_x}, {center_y}),', (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(self.color_image, f'World Coordinates: {camera_world_coords}', (x1, y1 - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                images = np.hstack((self.color_image, self.depth_colormap))
                cv2.imshow('RealSense Color and Depth Stream', images)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.stop()
            cv2.destroyAllWindows()



if __name__ == '__main__':
    camera = CameraStreamer()
    camera.run_object_detection()
