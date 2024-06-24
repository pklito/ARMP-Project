
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
import numpy as np
import os
from simple_pid import PID

# Append the parent directory of the current script's directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.CameraUtils.CameraStreamer import *
from src.Robot.RTDERobot import *
from src.CameraUtils.CameraFunctions import *


# Config
plate_center = (355, 220)
wrist_3_balance = 269.47
"""
# a potential problem with using configurations is that the ball might keep on rolling
# even if we get to the goal configuration we're trying to center around. consider changing this!
"""
balanced_conf = [0.2144722044467926, -2.2630707226195277, -0.0021737099159508944, -0.8701519531062623, 1.3459954261779785, -1.5668462175599647]


ur5e_conf = np.deg2rad([-0.97, -87.86, 20.94, -61.44, -91.22, -0.05])

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

    positions = detect_object(color_image)
    if len(positions) == 0:
        print('No object detected!')
        return None

    x1, y1, x2, y2 = positions[0]
    ball_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

    return plate_center[0] - ball_center[0], plate_center[1] - ball_center[1]

robot = RTDERobot()

keep_moving = True
while keep_moving:
    if not robot.getState():
        break

    current_config = balanced_conf.copy()

    errors = get_error()
    if errors is None:
        robot.sendWatchdog(0)
        continue

    error_x, error_y = errors
    current_config[5] += pid_controller_x(error_x)
    current_config[3] += pid_controller_y(error_y)

    robot.sendConfig(current_config)

    robot.sendWatchdog(1)
