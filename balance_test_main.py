
from time import time
from math import fmod
import numpy as np
from realsense_camera.CameraStreamer import *
from RTDERobot import *
from collections import deque
from simple_pid import PID

from realsense_camera.localization import get_aruco_corners, get_object, getPixelOnPlane
from realsense_camera.constants import *

def get_ball_position(color_image):
    if color_image is None or color_image.size == 0:
        return None

    positions = detect_ball(color_image)
    if len(positions) == 0:
        return None

    ball_center, radius = positions[0]

    ids, corners = get_aruco_corners(color_image)
    if ids is None:
        return None

    object_pts = np.array([a for i in ids for a in get_object([i[0]])], dtype=np.float32)
    pixel_pts = np.array([c for aruco in corners for c in aruco[0]], dtype=np.float32)
    if(len(object_pts) != len(pixel_pts)):
        return None

    if pixel_pts.ndim == 3 and pixel_pts.shape[1] == 1 and pixel_pts.shape[2] == 4:
        pixel_pts = pixel_pts[:, 0, :]

    if object_pts.size == 0 or pixel_pts.size == 0:
        return None

    ret, rvec, tvec = cv2.solvePnP(object_pts, pixel_pts, CAMERA_MATRIX, CAMERA_DIST_COEFF)

    if ret:
        return getPixelOnPlane((ball_center[0], ball_center[1]),rvec,tvec)
    return None


print("initializing Robots")
camera = CameraStreamer()
task_robot = RTDERobot("192.168.0.11",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

pid_controller_x = PID(Kp=1.5, Ki=0, Kd=0.5)
pid_controller_x.setpoint = 0

pid_controller_y = PID(Kp=0.6, Ki=0, Kd=0.1)
pid_controller_y.setpoint = 0

plate_center = (0, 0, 0)

print("start!")
keep_moving = True
not_started = True

initial_pos = [0.44, -2.84, 0.0, -0.26, 1.12, -1.58]
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

    current_task_config = task_state.target_q
    current_cam_config = cam_state.target_q

    ball_position = get_ball_position(color_image)
    if ball_position is None:
        continue
    error = ball_position - plate_center

    pos = initial_pos.copy()
    pos[5] += pid_controller_x(-error[1])
    pos[3] += pid_controller_y(-error[0])
    print(error)
    #task_robot.sendConfig(pos)


