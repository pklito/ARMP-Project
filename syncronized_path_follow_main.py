from robot_utils.PathFollow import PathFollow, PathFollowStrict
import sys
import os
from RTDERobot import RTDERobot
from robot_utils.constants import *
task_path = [[0.797, -2.788, -0.017, -0.379, -0.055, -1.566],
       [0.351, -2.031, -0.015, -1.383, 1.233, -1.548],
       [0.291, -1.088, -0.012, -2.096, 1.335, -1.574]]

camera_path = [[],[],[]]
# path = [[0.0, -1.571, 0.0, -1.571, 0.0, 0.0],
#         [0.797, -2.788, -0.017, -0.379, -0.055, -1.566]]

task_follower = PathFollowStrict(task_path, TASK_PATH_LOOKAHEAD, TASK_EDGE_CUTOFF)

task_robot = RTDERobot("192.168.0.12",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

timer_print = 0
keep_moving = True
while keep_moving:
    task_state = task_robot.getState()
    cam_state = camera_robot.getState()
    if not task_state or not cam_state:
        break

    if not task_state.output_int_register_0 or not cam_state.output_int_register_0:
        timer_print += 1
        if timer_print % 60 == 1:
            print(" waiting for ", "task robot" if not task_state.output_int_register_0 else "", " camera_robot" if not cam_state.output_int_register_0 else "")
            print("task config:", [round(q,2) for q in task_state.target_q])
            print("camera config:", [round(q,2) for q in cam_state.target_q])
        continue
    elif not_started:
        not_started = True

    task_robot.sendWatchdog(1)
    camera_robot.sendWatchdog(1)

    current_task_config = task_state.target_q
    current_cam_config = cam_state.target_q
    lookahead_config, target_edge, target_t = task_follower.getLookaheadAndT(current_task_config)
    task_follower.updateCurrentEdge(current_task_config)
    index = task_follower.current_edge


    task_robot.sendConfig(lookahead_config)



