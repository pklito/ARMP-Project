
from src.Robot.RTDERobot import RTDERobot

import sys
import os
import numpy as np
from simple_pid import PID

# Append the parent directory of the current script's directory to sys.path
from src.CameraUtils.CameraStreamer import CameraStreamer
from src.CameraUtils.localization import get_aruco_corners, get_object, getPixelOnPlane, get_obj_pxl_points
from src.CameraUtils.cameraConstants.constants import *
from src.CameraUtils.CameraFunctions import *
from src.MotionUtils.motionConstants.constants import *
from src.Robot.RTDERobot import *
import src.MotionUtils.PathFollow as PathFollow

import sys
import os
import time
# Append the parent directory of the current script's directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.Robot.RTDERobot import RTDERobot

new_robot_path = []
new_cam_path = []
task_path = [[1.743, -1.458, 2.261, -3.928, -1.598, 1.558],
             [1.743, -1.458, 2.261, -3.928, -1.598, 1.558],
            [0.676, -1.713, 2.538, -3.928, -1.598, 1.558],
            [0.676, -1.872, 2.104, -3.354, -1.598, 1.558],
            [0.6, -0.876, -1.509, -0.76, -1.598, 1.558],
            [-1.479, -1.444, -1.181, -0.486, -1.598, 1.558],
            [-3.038, -1.656, 0.346, -1.816, -1.598, 1.558],
            [-3.449, -2.784, 2.257, -2.618, -1.598, 1.558]]

camera_path = [
  [0.0, -1.554, -0.0, -0.539, -1.145, -0.0],
  [0.0, -1.554, -0.0, -0.539, -1.145, -0.0],
  [0.043, -1.338, 0.743, -1.646, -1.317, -0.247],
  [0.043, -1.506, 0.696, -1.703, -1.31, -0.42],
  [-0.225, -1.288, 0.22, -1.341, -1.152, -0.414],
  [-0.228, -1.532, 0.22, -1.065, -1.483, -0.414],
  [-0.409, -2.022, 0.673, -0.987, -1.832, -0.415],
  [-0.408, -2.289, 1.152, -1.038, -1.574, -0.139]]

from src.CameraUtils.CameraRun import _localization_detection
path_index = 0
if __name__ == '__main__':
    camera = CameraStreamer(width=1280, height=720)
    robot = RTDERobot(ROBOT_HOST='192.168.0.12',config_filename="./control_loop_configuration.xml")
    cam_robot = RTDERobot(ROBOT_HOST='192.168.0.10',config_filename="./control_loop_configuration.xml")
    last_time = 0
    start_time = time.time()
    while True:
        state = robot.getState()
        camstate = cam_robot.getState()
        if not state or not camstate:
            continue
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            path_index += 1
            print("new index", path_index)
        if key == ord('x'):
            path_index -= 1
            print("new index", path_index)
        if key == ord('f'):
            new_robot_path.append(state.actual_q)
            new_cam_path.append(camstate.actual_q)
        if key == ord('q'):
            print("task_path = ", new_robot_path)
            print("camera_path = ", new_cam_path)

        path_index = path_index % len(task_path)

        _localization_detection(camera)

        robot.sendConfig(PathFollow.getClampedTarget(state.actual_q,  task_path[path_index], 0.1))
        cam_robot.sendConfig(PathFollow.getClampedTarget(camstate.actual_q, camera_path[path_index], 0.1))
        cam_robot.sendWatchdog(1)
        robot.sendWatchdog(1)
