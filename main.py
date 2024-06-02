#!/usr/bin/env python
# Copyright (c) 2016-2022, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
sys.path.append("..")
import logging
from time import time
from math import fmod
import numpy as np
from realsense_camera.CameraStreamer import *
from RTDERobot import *
from simple_pid import PID

# Config
plate_center = (338, 280)
"""
# a potential problem with using configurations is that the ball might keep on rolling
# even if we get to the goal configuration we're trying to center around. consider changing this!
"""
balanced_conf = [0.2144722044467926, -2.2630707226195277, -0.0021737099159508944, -0.8701519531062623, 1.3459954261779785, -1.5668462175599647]

ur5e_conf = np.deg2rad([-0.97, -87.86, 20.94, -61.44, -91.22, -0.05]) # determines the camera's position

# Settings
pid_controller_x = PID(Kp=0.0006, Ki=0, Kd=0.0003)
pid_controller_x.setpoint = 0


pid_controller_y = PID(Kp=0.0006, Ki=0, Kd=0.0003)
pid_controller_y.setpoint = 0

camera = CameraStreamer()

def get_error():
    color_image, depth_image, depth_frame, depth_map = camera.get_frames()
    if color_image.size == 0 or depth_image.size == 0:
        print("Can't receive frame (stream end?).")
        return None  # Return None to indicate an error state

    plate_positions = detect_plate(color_image)
    object_positions = detect_object(color_image)
    if len(object_positions) == 0 or len(plate_positions) == 0:  
        print('No object detected!')  
        return None
    
    p_x1, p_y1, p_x2, p_y2 = plate_positions[0]
    plate_center = (int((p_x1 + p_x2) / 2), int((p_y1 + p_y2) / 2)) # dynamic positioning of the plate
    print(f'Plate Center: {plate_center}')

    x1, y1, x2, y2 = object_positions[0]
    object_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
    print(f'Ball Center: {object_center}')

    return plate_center[0] - object_center[0], plate_center[1] - object_center[1]

def interpolate_path(start_conf, goal_conf, num_steps=100):
    path = []
    for i in range(num_steps + 1):
        alpha = i / num_steps
        intermediate_conf = (1 - alpha) * np.array(start_conf) + alpha * np.array(goal_conf)
        intermediate_conf[5] = pid_x_config
        intermediate_conf[3] = pid_y_config
        path.append(intermediate_conf)
    return path

num_steps = 100
start_conf = np.deg2rad([-0.97, -87.86, 20.94, -61.44, -91.22, -0.05])
pid_x_config = start_conf[5]
pid_y_config = start_conf[3]
goal_conf = np.deg2rad([30.0, -45.0, 20.0, -60.0, -90.0, 0.0])  # example goal position - change accordingly

path = interpolate_path(start_conf, goal_conf, num_steps) # naive path planning - TODO: use a more suffosticated algorithm
# also need to move the ur5e as to keep the plate and the ball within the camera's frame!
# to get the ur5e end effector position:
# 1. given the joint configuration of the UR3E
# 2. calculate the Forward Kinematics to get the position of the end effector which is holding the plate
# 3. now given the position of the plate, add a safety distance above it for the camera and calculate Inverse kinematics for the UR5E
# 4. after calculating the inverse kinematics for hte UR5E, make sure to execute the path in synchronization the UR3E

robot = RTDERobot()

keep_moving = True
while keep_moving:
    # if not robot.getState():
    #     break
    for waypoint in path:
        if not robot.getState():
            break
    
    # current_config = balanced_conf.copy()
        current_config = waypoint.copy()

        errors = get_error()
        if errors is None:
            robot.sendWatchdog(0)
            continue
        
        error_x, error_y = errors
        current_config[5] += pid_controller_x(error_x)
        pid_x_config = current_config[5]
        current_config[3] += pid_controller_y(error_y)
        pid_y_config = current_config[3]
        
        robot.sendConfig(current_config)

        robot.sendWatchdog(1)
    keep_moving = False # done executing the path!