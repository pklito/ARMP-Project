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
plate_center = (317, 356)
wrist_3_balance = 269.47
"""
# a potential problem with using configurations is that the ball might keep on rolling
# even if we get to the goal configuration we're trying to center around. consider changing this!
"""
balanced_conf = [0.44, -2.22, -0.0, -0.93, 1.08, -1.57] 

# Settings
pid_controller = PID(Kp=0.002, Ki=0, Kd=0.0008)
pid_controller.setpoint = 0


camera = None

def get_error():
    while True:
        color_image, depth_image, depth_frame, depth_map = camera.get_frames()
        if color_image.size == 0 or depth_image.size == 0:
            print("Can't receive frame (stream end?).")
            return None  # Return None to indicate an error state

        positions = detect_object(color_image)
        if len(positions) == 0:  
            print('No object detected!')
            time.sleep(1)   
            continue

        x1, y1, x2, y2 = positions[0]
        ball_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

        error = plate_center[0] - ball_center[0]
        return error
    
robot = RTDERobot()

keep_moving = True
while keep_moving:
    if not robot.getState():
        break
    
    current_config = balanced_conf.copy()

    current_config[5] += fmod(time()/6,1)
    robot.sendConfig(current_config)

    robot.sendWatchdog(1)