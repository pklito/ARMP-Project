
from time import time
from math import fmod
import numpy as np
from realsense_camera.CameraStreamer import *
from RTDERobot import *
import robot_utils.kinematicsUtils as fk
from collections import deque
from simple_pid import PID

print("initializing Robots")
camera = CameraStreamer()
task_robot = RTDERobot("192.168.0.11",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

pid_controller_x = PID(Kp=1.5, Ki=0, Kd=0.5)
pid_controller_x.setpoint = 0

pid_controller_y = PID(Kp=0.6, Ki=0, Kd=0.1)
pid_controller_y.setpoint = 0

plate_center = (0, 0)

print("start!")
keep_moving = True
not_started = True

initial_pos = [0.44, -2.84, 0.0, -0.26, 1.12, -1.58]
count = 0
while keep_moving:
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

    error = 0
    # Error getting code


    #
    
    pos = initial_pos.copy()
    pos[5] += pid_controller_x(-error[1])
    pos[3] += pid_controller_y(-error[0])
    print(error)
    task_robot.sendConfig(pos)


