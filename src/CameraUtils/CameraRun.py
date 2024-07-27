import cv2
import numpy as np
from src.CameraUtils.cameraConstants.constants import *
from src.CameraUtils.localization import get_aruco_corners, get_obj_pxl_points, getPixelOnPlane
from src.CameraUtils.CameraFunctions import _ball_hsv_mask, detect_and_draw_arucos, detect_ball, detect_object, get_world_position_from_camera

def _localization_detection(camera):
    color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics, is_new_image = camera.get_frames()
    if color_image is None or is_new_image == False:
        return True

    if color_image.size == 0:
        print("Can't receive frame (stream end?).")
        return True

    positions = detect_ball(color_image)
    if len(positions) == 0:
        ball_center, radius = None, None
    else:
        ball_center, radius = positions[0]

    ids, corners = get_aruco_corners(color_image)
    wcpoints = (-1,-1,-1)
    if ids is not None:
        object_pts, pixel_pts = get_obj_pxl_points([a[0] for a in ids.tolist()], corners)
        if(len(object_pts) != len(pixel_pts)):
            print("Error, sizes", len(object_pts),len(pixel_pts))
        else:
            if pixel_pts.ndim == 3 and pixel_pts.shape[1] == 1 and pixel_pts.shape[2] == 4:
                pixel_pts = pixel_pts[:, 0, :]

            if object_pts.size == 0 or pixel_pts.size == 0:
                return True
            ret, rvec, tvec = cv2.solvePnP(object_pts, pixel_pts, CAMERA_MATRIX, CAMERA_DIST_COEFF)

            if ret:
                if ball_center is not None:
                    wcpoint = getPixelOnPlane((ball_center[0], ball_center[1]),rvec,tvec)
                    print([round(a,3) for a in wcpoint])

                cv2.drawFrameAxes(color_image, CAMERA_MATRIX, CAMERA_DIST_COEFF, rvec, tvec, 0.026, 2)

    color = (0, 255, 0)  # Green for ball
    if ball_center is not None:
        cv2.circle((color_image), ball_center, radius, color, 2) # the enclosing circle
        cv2.circle((color_image), ball_center, 2 ,color,2) # a dot in the middle of the circle
        cv2.putText((color_image), f'Ball Center: ({ball_center[0]}, {ball_center[1]}),', (ball_center[0], ball_center[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    if depth_colormap is not None:
        images = np.hstack((color_image, depth_colormap))
    else:
        images = color_image

    cv2.imshow('RealSense Stream', images)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        camera.stop()
        cv2.destroyAllWindows()

def localization_detection(camera):
        try:
            while True:
                ret = _localization_detection(camera)
                if not ret:
                    continue
        finally:
            pass

def ball_hsv_mask(camera):
    try:
        while True:
            color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics, is_new_image = camera.get_frames()
            if color_image is None or not is_new_image:
                continue

            mask = _ball_hsv_mask(color_image)
            cv2.imshow("mask", mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()

def ball_hsv_mask_grayscale(camera):
    try:
        while True:
            color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics, is_new_image = camera.get_frames()
            if color_image is None or not is_new_image:
                continue

            mask = _ball_hsv_mask(color_image)
            mask3 = np.zeros_like(color_image)
            mask3[:, :, 0] = mask
            mask3[:, :, 1] = mask
            mask3[:, :, 2] = mask
            overlay = cv2.bitwise_and(mask3, color_image)
            background = cv2.bitwise_and(255-mask3, color_image)
            background = cv2.cvtColor(cv2.cvtColor(background, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
            #mask = cv2.multiply(color_image,cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))
            cv2.imshow("mask", overlay+background)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()

def run_object_detection(camera):
        try:
            while True:
                color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics, is_new_image = camera.get_frames()

                if color_image is None or depth_colormap is None or depth_frame is None:
                    continue

                object_bounding_boxes = detect_object(color_image)
                plate_bounding_boxes = [] # detect_plate(color_image)

                # Combine bounding boxes for both ball and plate
                all_bounding_boxes = object_bounding_boxes + plate_bounding_boxes

                for bbox in all_bounding_boxes:
                    x1, y1, x2, y2 = bbox
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    depth_value = depth_frame.get_distance(center_x, center_y)
                    distance_meters = depth_value * 1000  # Convert to millimeters
                    camera_world_coords = get_world_position_from_camera(center_x, center_y, depth_frame_input=depth_frame)

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
            camera.stop()
            cv2.destroyAllWindows()

def drawBothFrames(camera):
    camera.stream()

def draw_arucos(camera):
    color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics, was_new = camera.get_frames()
    detect_and_draw_arucos(color_image)
