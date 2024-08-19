task_path = [[0.3, -1.059, -1.229, -0.874, 1.716, 1.523],
             [-0.406, -1.349, -1.227, -0.598, 1.3, 1.575],
             [-0.619, -2.458, -1.248, 0.597, 1.566, 1.555],
             [0.237, -1.844, -1.927, 0.597, 1.566, 1.555]]

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


if __name__ == '__main__':
    robot = RTDERobot(ROBOT_HOST='192.168.0.12',config_filename="./control_loop_configuration.xml")
    last_time = 0
    start_time = time.time()
    while True:
        state = robot.getState()
        if not state:
            continue

        robot.sendConfig(PathFollow.getClampedTarget(state.actual_q, task_path[3], 0.1))
        robot.sendWatchdog(1)
