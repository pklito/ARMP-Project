
from time import time
from math import fmod
import numpy as np
from realsense_camera.CameraStreamer import *
from RTDERobot import *
from collections import deque
from simple_pid import PID

from realsense_camera.localization import get_aruco_corners, get_object, getPixelOnPlane
from realsense_camera.constants import *

DEBUG = True
def get_ball_position(color_image):
    if color_image is None or color_image.size == 0:
        return None

    positions = detect_ball(color_image)
    if len(positions) == 0:
        if DEBUG:
            print("Ball not detected")
        return None

    ball_center, radius = positions[0]

    ids, corners = get_aruco_corners(color_image)
    if ids is None:
        if DEBUG:
            print("aruco corners not detected")
        return None

    object_pts, pixel_pts = get_obj_pxl_points(ids, corners)

    if(len(object_pts) != len(pixel_pts)):
        if DEBUG:
            print("[Error] obj points, pixel points, ids: ", len(object_pts), len(pixel_pts), ids.tolist())
        return None

    if pixel_pts.ndim == 3 and pixel_pts.shape[1] == 1 and pixel_pts.shape[2] == 4:
        pixel_pts = pixel_pts[:, 0, :]

    if object_pts.size == 0 or pixel_pts.size == 0:
        return None

    ret, rvec, tvec = cv2.solvePnP(object_pts, pixel_pts, CAMERA_MATRIX, CAMERA_DIST_COEFF)

    if ret:
        return getPixelOnPlane((ball_center[0], ball_center[1]),rvec,tvec)
    if DEBUG:
            print("solvePNP failed!")
    return None


print("initializing Robots")
camera = CameraStreamer()
signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, camera))
task_robot = RTDERobot("192.168.0.12",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

pid_controller_x = PID(Kp=2, Ki=0, Kd=0.0)
pid_controller_x.setpoint = 0

pid_controller_y = PID(Kp=1.5, Ki=0, Kd=0.0)
pid_controller_y.setpoint = 0

plate_center = (0, 0, 0)

print("start!")
keep_moving = True
not_started = True

initial_pos = [-0.129, -1.059, -1.229, -0.875, 1.716, 1.523]
count = 0
while keep_moving:
    color_image, _, _, _, _ = camera.get_frames()
    task_state = task_robot.getState()
    cam_state = camera_robot.getState()

    if not task_state.output_int_register_0:
        print("Not started yet", [round(q,2) for q in task_state.target_q])
        continue
    elif not_started:
        not_started = True

    if not task_state or not cam_state:
        break
    task_robot.sendWatchdog(1)
    camera_robot.sendWatchdog(1)

    if color_image is None or color_image.size == 0:
        continue

    # cv2.waitKey(1)
    # cv2.imshow("name", color_image)

    current_task_config = task_state.target_q
    current_cam_config = cam_state.target_q

    ball_position = get_ball_position(color_image)
    if ball_position is None:
        continue
    error = ball_position - plate_center

    pos = initial_pos.copy()
    pos[5] += pid_controller_x(error[0])
    pos[3] += pid_controller_y(-error[1])
    print([round(a,3) for a in error])
    task_robot.sendConfig(pos)


