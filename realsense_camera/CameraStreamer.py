import cv2
import pyrealsense2 as rs
import numpy as np
from PIL import Image


def calculate_similarity(color1, color2):
    return np.sum(np.abs(color1 - color2))

# Function to perform object detection on a frame
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
        
class CameraStreamer:
    def __init__(self):
        # Initialize RealSense camera pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
    
    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None, None

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image, depth_colormap

    def stop(self):
        self.pipeline.stop()
        
    def run_object_detection(self):
        try:
            while True:
                color_image, depth_image, depth_colormap = self.get_frames()

                if color_image is None or depth_image is None:
                    continue

                bounding_boxes = detect_object(color_image)
                for detected_ball in bounding_boxes:
                    x1, y1, x2, y2 = detected_ball
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    depth_value = depth_image.get_distance(center_x, center_y)
                    distance_meters = depth_value * 1000  # Convert to millimeters
                    
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(color_image, f'Distance: {distance_meters:.2f} mm, Center: ({center_x}, {center_y})', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                images = np.hstack((color_image, depth_colormap))
                cv2.imshow('RealSense Color and Depth Stream', images)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.stop()
            cv2.destroyAllWindows()



if __name__ == '__main__':
    camera = CameraStreamer()
    camera.run_object_detection()