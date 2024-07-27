from turtle import color
import cv2
import numpy as np
from src.CameraUtils.cameraConstants.constants import CAMERA_MATRIX, CAMERA_DIST_COEFF
from src.CameraUtils.localization import get_aruco_corners, get_obj_pxl_points, getPixelOnPlane

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


def custom_detect_arucos(color_image):
    """Detects arucos       [!] uses cv2.imwrite("aruco_detected.jpg")"""

    if color_image is None or color_image.size == 0:
        print("Error: The image is empty or not loaded correctly.")
        return None, None

    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    arucoParams = cv2.aruco.DetectorParameters()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParams)
    if ids is not None:
        color_image_with_markers = cv2.aruco.drawDetectedMarkers(color_image.copy(), corners, ids)

        aruco_corners = []
        for id, corner in zip(ids, corners):
            print("id: " + str(id))
            c = corner[0]
            aruco_corner = [(c[0][0], c[0][1]), (c[1][0], c[1][1]), (c[2][0], c[2][1]), (c[3][0], c[3][1])]
            aruco_corners.append(aruco_corner)
            for point in aruco_corner:
                cv2.circle(color_image_with_markers, (int(point[0]), int(point[1])), 3 ,(0,255,0),2)
        key = cv2.waitKey(1)
        cv2.imwrite("aruco_detected.png", color_image_with_markers)
        if key == ord('q'):
            return None, None
        return aruco_corners, color_image_with_markers
    else:
        print("No arucos detected")
        color_image_with_markers = color_image.copy()
    return None, color_image_with_markers

def drawAxesCustom(color_image, axis_length = 0.05):
    ids, corners = get_aruco_corners(color_image)
    if ids is None:
        exit(1)
    object_pts, pixel_pts = get_obj_pxl_points([a[0] for a in ids.tolist()], corners)

    if(len(object_pts) != len(pixel_pts)):
        exit(1)

    if pixel_pts.ndim == 3 and pixel_pts.shape[1] == 1 and pixel_pts.shape[2] == 4:
        pixel_pts = pixel_pts[:, 0, :]

    if object_pts.size == 0 or pixel_pts.size == 0:
        exit(1)

    ret, rvec, tvec = cv2.solvePnP(object_pts, pixel_pts, CAMERA_MATRIX, CAMERA_DIST_COEFF)

    # Draw axes frames on the image
    axis_points, _ = cv2.projectPoints(np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]), rvec, tvec, CAMERA_MATRIX, CAMERA_DIST_COEFF)
    origin = tuple(map(int, axis_points[0].ravel()))
    x_axis = tuple(map(int, axis_points[1].ravel()))
    y_axis = tuple(map(int, axis_points[2].ravel()))
    z_axis = tuple(map(int, axis_points[3].ravel()))

    cv2.line(color_image, origin, x_axis, (0, 0, 255), 2)
    cv2.line(color_image, origin, y_axis, (0, 255, 0), 2)
    cv2.line(color_image, origin, z_axis, (255, 0, 0), 2)
    return color_image

ball_image = cv2.imread('Ball_before_detect.png')
aruco_image = cv2.imread('aruco_image.png')

bounding_circles = detect_ball(ball_image)

for circle in bounding_circles:
    center, radius = circle
    cv2.circle(ball_image, center, radius, (0, 255, 0), 2)
    cv2.circle(ball_image, center, 3, (255, 0, 0), -1)

cv2.imwrite("Ball_Detected.png", ball_image)

_ = custom_detect_arucos(aruco_image)
aruco_image = cv2.imread('aruco_image.png')
new_image = drawAxesCustom(aruco_image, axis_length = 0.05)

cv2.imwrite("Axes_Frames.png", new_image)
