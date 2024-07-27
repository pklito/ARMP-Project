
import cv2
import numpy as np
import pyrealsense2 as rs
from src.CameraUtils.cameraConstants.constants import CAMERA_MATRIX, CAMERA_DIST_COEFF
from src.CameraUtils.localization import get_aruco_corners, get_obj_pxl_points, getPixelOnPlane
import logging
logger = logging.getLogger("LogGenerator")
"""These are functions which use the color buffer that do image recognition
   Namely, aruco detection, ball detection, plate detection, etc."""
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

def _ball_hsv_mask(frame):
    frame = cv2.GaussianBlur(frame, (17, 17), 0)
    kernel = np.ones((5, 5), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)

    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_limit1, upper_limit1 = (165, 136, 141), (180,  255, 255)
    #lower_limit1, upper_limit1 = (0, 154, 146), (9,  255, 255)
    #lower_limit2, upper_limit2 = (176, 154, 146), (179,  255, 255) Realsense!!
    lower_limit2, upper_limit2 = (256, 255, 255), (255,  255, 255)       # fake camera

    mask1 = cv2.inRange(hsv_image, lower_limit1, upper_limit1)
    mask2 = cv2.inRange(hsv_image, lower_limit2, upper_limit2)
    mask = cv2.bitwise_or(mask1, mask2)
    return mask

def detect_ball(frame):
    mask = _ball_hsv_mask(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    bounding_circles = []
    for contour in contours:
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        bounding_circles.append((center, radius))
    return bounding_circles


def mark_arucos(color_image):
        """Detects arucos       [!] uses cv2.imwrite("aruco_detected.jpg")"""

        if color_image is None or color_image.size == 0:
            print("Error: The image is empty or not loaded correctly.")
            return None, None

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        arucoParams = cv2.aruco.DetectorParameters()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParams)
        if ids is not None:
            color_image_with_markers = cv2.aruco.drawDetectedMarkers(color_image.copy(), corners, ids)

            centers = []
            for id, corner in zip(ids, corners):
                print("id: " + str(id))
                c = corner[0]
                center = (c[:, 0].mean(), c[:, 1].mean())
                centers.append(center)
                cv2.circle(color_image_with_markers, (int(center[0]), int(center[1])), 3 ,(0,255,0),2)
            avg_center = np.mean(centers, axis=0)
            cv2.circle(color_image_with_markers, (int(avg_center[0]), int(avg_center[1])), 3 ,(255,0,0),2)
            return avg_center, color_image_with_markers
        else:
            print("No arucos detected")
            color_image_with_markers = color_image.copy()
        return None, color_image_with_markers


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


def get_world_position_from_camera(pixel_x, pixel_y, depth_frame_input):
        depth = depth_frame_input.get_distance(pixel_x, pixel_y)
        depth_intrinsics = depth_frame_input.profile.as_video_stream_profile().intrinsics

        # Convert pixel coordinates to world coordinates (from the Camera's Perspective!)
        ball_position = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [pixel_x, pixel_y], depth)
        return np.array(ball_position)


def get_ball_position(color_image, DEBUG = False):
    if color_image is None or color_image.size == 0:
        return None

    positions = detect_ball(color_image)
    if len(positions) == 0:
        logger.error("ball position not detected")
        return None

    ball_center, radius = positions[0]
    logger.debug(f"Ball position: {ball_center}, radius: {radius}, total_found: {len(positions)}")
    ids, corners = get_aruco_corners(color_image)
    if ids is None:
        logger.error("no aruco corners found")
        return None
    logger.debug(f"aruco ids: {ids}")
    object_pts, pixel_pts = get_obj_pxl_points([a[0] for a in ids.tolist()], corners)

    if(len(object_pts) != len(pixel_pts)):
        logger.error(f"[Error] obj points {len(object_pts)}, pixel points {len(pixel_pts)}, ids: {ids.tolist()}")
        return None

    if pixel_pts.ndim == 3 and pixel_pts.shape[1] == 1 and pixel_pts.shape[2] == 4:
        pixel_pts = pixel_pts[:, 0, :]

    if object_pts.size == 0 or pixel_pts.size == 0:
        return None

    ret, rvec, tvec = cv2.solvePnP(object_pts, pixel_pts, CAMERA_MATRIX, CAMERA_DIST_COEFF)

    if ret:
        logger.error("SolvePnP failed")
        return getPixelOnPlane((ball_center[0], ball_center[1]),rvec,tvec)
    return None
