import cv2
import pyrealsense2 as rs
import numpy as np
from PIL import Image
from detectedObjects import *

# Function to perform object detection on a frame
def detect_object(frame):

    frame = cv2.GaussianBlur(frame, (17, 17), 0)
    kernel = np.ones((5, 5), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)

    # Convert to HSV color space
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_limit1, upper_limit1 = (0, 154, 146), (9,  255, 255)
    lower_limit2, upper_limit2 = (176, 154, 146), (179,  255, 255)

    mask1 = cv2.inRange(hsv_image, lower_limit1, upper_limit1)
    mask2 = cv2.inRange(hsv_image, lower_limit2, upper_limit2)
    mask = cv2.bitwise_or(mask1, mask2)

    mask_pil = Image.fromarray(mask)
    bbox = mask_pil.getbbox()

    if bbox is None:
        return None

    return bbox

def gen_frames():
    # Initialize RealSense camera pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            color_image = np.asanyarray(color_frame.get_data())
            
            detected_ball = detect_object(color_image)  # Call your object detection function
            if detected_ball is not None:
                x1, y1, x2, y2 = detected_ball  # Extract bounding box coordinates - top left + bottom right?
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                depth_value = depth_frame.get_distance(center_x, center_y)
                distance_meters = depth_value * 1000  # Convert to millimeters

                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Display the distance value on the color image
                cv2.putText(color_image, f'Distance: {distance_meters:.2f} mm', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Combine color image with colormap
            images = np.hstack((color_image, depth_colormap))
            cv2.imshow('RealSense Color and Depth', images)

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()



if __name__ == '__main__':
    gen_frames()