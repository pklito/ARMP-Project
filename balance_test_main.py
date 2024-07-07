import numpy as np
import os
from time import time
from math import fmod
from collections import deque
from simple_pid import PID
import matplotlib.pyplot as plt
import sys
from src.LogGenerator import LoggerGenerator
# Append the parent directory of the current script's directory to sys.path
from src.CameraUtils.CameraStreamer import CameraStreamer, signal
from src.Robot.RTDERobot import *
from src.CameraUtils.localization import get_aruco_corners, get_object, getPixelOnPlane, get_obj_pxl_points
from src.CameraUtils.cameraConstants.constants import *
from src.CameraUtils.CameraFunctions import *


debug_plot = []

DEBUG = True
logger = LoggerGenerator("debug.log", consoleLevel=10)

logger.info("Connecting to camera")
camera = CameraStreamer(no_depth=True)
# signal.signal(signal.SIGINT, lambda sig, frame: my_signal_handler(sig, frame, debug_plot))
task_robot = RTDERobot("192.168.0.12",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

pid_controller_x = PID(Kp=0.9, Ki=0, Kd=0.361,output_limits=(-0.4,0.4))
pid_controller_x.setpoint = 0

pid_controller_y = PID(Kp=0.7, Ki=0, Kd=0.2,output_limits=(-0.3,0.3))
pid_controller_y.setpoint = 0

plate_center = (0, 0, 0)

CAMERA_FAILED_MAX = 5

logger.info("starting")
keep_moving = True
not_started = True
start_time = time()
# initial_pos = [-0.16, -2.78, 2.26, -2.62, -1.6, 1.56]
initial_pos = [0, -2.78, 2.26, -2.638, -1.6, 1.567]

camera_failed_counter = 0
# signal.signal(signal.SIGINT, lambda sig, frame: my_signal_handler(sig, frame, debug_plot, camera)) # may not work properly
while keep_moving:
    color_image, _, _, _, _, Is_image_new = camera.get_frames()
    task_state = task_robot.getState()
    cam_state = camera_robot.getState()

    if not task_state or not cam_state:
        print("robot disconnected!")
        break

    if not task_state.output_int_register_0:
        print("Not started yet", [round(q,2) for q in task_state.target_q])
        continue
    elif not_started:
        not_started = True

    if not task_state or not cam_state:
        break
    task_robot.sendWatchdog(1)
    camera_robot.sendWatchdog(1)

    if not Is_image_new:
        #print("Old image!")
        continue
    if color_image is None or color_image.size == 0:
        continue

    # cv2.waitKey(1)
    # cv2.imshow("name", color_image)
    current_task_config = task_state.target_q
    current_cam_config = cam_state.target_q

    ball_position = get_ball_position(color_image,DEBUG=False)
    error = (0,0)
    if ball_position is None:
        camera_failed_counter += 1
        if camera_failed_counter < CAMERA_FAILED_MAX:
            continue
        #print("camera failed for ", camera_failed_counter, " frames")
    else:
        camera_failed_counter = 0
        error = ball_position - plate_center

    debug_plot.append((time()-start_time, error[0]))

    pos = initial_pos.copy()
    pos[5] += pid_controller_x(-error[1])
    pos[3] += pid_controller_y(error[0])
    #print([round(a,3) for a in error])
    logger.debug([round(a,3) for a in error])
    task_robot.sendConfig(pos)
