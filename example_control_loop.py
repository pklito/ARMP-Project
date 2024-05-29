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

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from time import time
from math import fmod

import numpy as np
from CameraController import *
from Robots import *
from realsense_camera import stream_and_detect



def clamp(value, min, max):
    if value < min:
        return min
    if value > max:
        return max
    return value

# Config
plate_center = (317, 356)
wrist_3_balance = 269.47
balanced_conf = [0.44, -2.22, -0.0, -0.93, 1.08, -1.57]



# Settings
max_error = 50
kp = -0.08/60
        
# logging.basicConfig(level=logging.INFO)

ROBOT_HOST = "192.168.0.11"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

old_setp2 = [34.03, -115.56, -64.3, 0.2, 56.23, 269.47]
# # Setpoints to move the robot to
# setp1 = [0, -3.14 / 2, 0, -3.14 / 2, 0, 0]
# setp2 = [(3.1415926*a)/180 for a in old_setp2]

setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0


def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list


def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp

def full_print(state,move_completed = False):
    print(watchdog.input_int_register_0, move_completed)
    
    print([float('%.2f' % a) for a in state.target_q],[float('%.2f' % a) for a in state.target_qd],state.output_int_register_0)

# start data synchronization
if not con.send_start():
    sys.exit()


camera = CameraWrapper()

def get_error():
    ret, frame = camera.cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        exit()
        
    position = stream_and_detect.detect_object(frame)[0]
    if(position is None):
        return None
    
    x1,y1,x2,y2 = position
    ball_center = (int((x1+x2)/2), int((y1+y2)/2))
    
    error = plate_center[0] - ball_center[0]
    print("error: ", plate_center[0] - ball_center[0])

    return error
    
        
# control loop
move_completed = True
while keep_running:
    # receive the current state
    state = con.receive()
    if state is None:
        break
    full_print(state)
    # do something...
    if move_completed and state.output_int_register_0 == 1:
        move_completed = False
        
        new_setp = balanced_conf
        error = get_error()
        if(error is None):
            continue
        new_setp[5] += kp * error
        
        list_to_setp(setp, new_setp)
        print("New pose = " + str(new_setp))
        # send new setpoint
        con.send(setp)
        watchdog.input_int_register_0 = 1
    elif not move_completed and state.output_int_register_0 == 0:
        print("Move to confirmed pose = " + str(state.target_q))
        move_completed = True
        watchdog.input_int_register_0 = 0

    # kick watchdog
    con.send(watchdog)

con.send_pause()

con.disconnect()
