from robot_utils.PathFollow import PathFollow, PathFollowStrict
import sys
import os
from RTDERobot import RTDERobot
from robot_utils.constants import *
path = [[0.797, -2.788, -0.017, -0.379, -0.055, -1.566],
       [0.351, -2.031, -0.015, -1.383, 1.233, -1.548],
       [0.291, -1.088, -0.012, -2.096, 1.335, -1.574]]

# path = [[0.0, -1.571, 0.0, -1.571, 0.0, 0.0],
#         [0.797, -2.788, -0.017, -0.379, -0.055, -1.566]]

pathfollower = PathFollowStrict(path, TASK_PATH_LOOKAHEAD, TASK_EDGE_CUTOFF)
task_robot = RTDERobot("192.168.0.11",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

keep_moving = True
while keep_moving:
    task_state = task_robot.getState()
    cam_state = camera_robot.getState()
    if not task_state or not cam_state:
        break
    task_robot.sendWatchdog(1)
    camera_robot.sendWatchdog(1)

    current_task_config = task_state.target_q
    current_cam_config = cam_state.target_q
    lookahead_config = pathfollower.getClampedLookaheadConfig(current_task_config)
    pathfollower.updateCurrentEdge(current_task_config)
    index = pathfollower.current_edge
    print(lookahead_config - current_task_config, index)

    task_robot.sendConfig(lookahead_config)



