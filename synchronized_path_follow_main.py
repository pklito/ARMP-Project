import sys
import os

# Append the parent directory of the current script's directory to sys.path
from src.MotionUtils.motionConstants.constants import *
from src.Robot.RTDERobot import *
import src.MotionUtils.PathFollow as PathFollow
import numpy as np

"""Path follower that maintains the camera runs the same path points as the task robot.
    Requires both robots to use 'rtde_synced_servoj.urp'"""
# task_path = [[0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
#                [0.0, -1.97, 0.0, -0.91, -0.98, -0.0],
#                [0.08, -2.13, 0.97, -1.99, 0.1, -0.0],
#                [0.07, -1.94, 1.91, -2.97, -0.85, -0.16]]

# camera_path = [[0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
#                [0.75, -2.59, 0.77, -1.55, 0.99, 0.0],
#                [0.08, -2.13, 0.97, -1.99, 0.1, -0.0],
#                [-0.9, -2.62, 0.77, -1.55, 0.99, 0.0]]

# task_path = [[1.743, -1.458, 2.261, -3.928, -1.598, 1.558],
#              [1.743, -1.458, 2.261, -3.928, -1.598, 1.558],
#             [0.676, -1.713, 2.538, -3.928, -1.598, 1.558],
#             [0.676, -1.872, 2.104, -3.354, -1.598, 1.558],
#             [0.6, -0.876, -1.509, -0.76, -1.598, 1.558],
#             [-1.479, -1.444, -1.181, -0.486, -1.598, 1.558],
#             [-3.038, -1.656, 0.346, -1.816, -1.598, 1.558],
#             [-3.449, -2.784, 2.257, -2.618, -1.598, 1.558]]

# camera_path = [
#   [0.0, -1.554, -0.0, -0.539, -1.145, -0.0],
#   [0.0, -1.554, -0.0, -0.539, -1.145, -0.0],
#   [0.043, -1.338, 0.743, -1.646, -1.317, -0.247],
#   [0.043, -1.506, 0.696, -1.703, -1.31, -0.42],
#   [-0.225, -1.288, 0.22, -1.341, -1.152, -0.414],
#   [-0.228, -1.532, 0.22, -1.065, -1.483, -0.414],
#   [-0.409, -2.022, 0.673, -0.987, -1.832, -0.415],
#   [-0.408, -2.289, 1.152, -1.038, -1.574, -0.139]]

# task_path = [[0.3, -1.059, -1.229, -0.81, 1.571, 1.571] ,
#         [0.3, -1.059, -1.229, -0.81, 1.571, 1.571] ,
#         [-0.406, -1.349, -1.227, -0.522, 1.571, 1.571] ,
#         [-0.619, -2.458, -1.248, 0.608, 1.571, 1.571] ,
#         [0.237, -1.844, -1.927, 0.673, 1.571, 1.571] ,
#         [0.3, -1.059, -1.229, -0.81, 1.571, 1.571] ,
#         ]

# camera_path = [[-0.03, -1.745, 0.1, -0.572, -1.753, 0.0],
#                [-0.03, -1.745, 0.1, -0.572, -1.753, 0.0],
#                 [0.681, -1.67, 0.191, -0.733, -1.753, 0.0],
#                [1.24, -1.981, 1.089, -1.144, -1.753, 0.319],
#                [0.937, -2.381, 1.038, -0.573, -2.043, 0.445],
#                [-0.03, -1.745, 0.1, -0.572, -1.753, 0.0]]

task_path = [[-1.254, -0.182, -1.686, -1.23, 1.571, 1.571] ,
        [-1.254, -0.182, -1.686, -1.23, 1.571, 1.571] ,
        [0.1, -0.188, -1.712, -1.198, 1.571, 1.571] ,
        [-1.561, -1.446, -2.057, 0.405, 1.571, 1.571] ,
        [-1.542, -2.684, -0.751, 0.337, 1.571, 1.571] ,
        ]

camera_path = [
                [-0.105, -2.148, 1.144, -1.357, -1.609, 0.111],
                [-0.105, -2.148, 1.144, -1.357, -1.609, 0.111],
                [-0.104, -2.13, 1.144, -1.354, -1.406, 0.0],
                [1.141, -1.252, 0.971, -1.682, -2.777, 0.087],
                [1.219, -1.186, 0.934, -2.086, -2.372, 0.086]
]

task_follower = PathFollow.PathFollowStrict(task_path, TASK_PATH_LOOKAHEAD, TASK_EDGE_CUTOFF)

from src.LogGenerator import LoggerGenerator
logger = LoggerGenerator(logfile=f"logs/synchronized_path_follow_checks.log", consoleLevel=20)

task_robot = RTDERobot("192.168.0.12",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

def toView(conf, end = "\n"):
    return [round(a, 2) for a in conf]

# TEMP
def shortenLookahead(lookahead, other_robot_loc, other_robot_target):
    """decrease lookahead based on the other robots position
       [!] DO NOT DECREASE CLAMP!!! otherwise both robots will simply deadlock"""
    other_distance_squared = np.dot(other_robot_loc - other_robot_target, other_robot_loc - other_robot_target)
    shorten = 0.75 / (other_distance_squared + 0.001)
    return PathFollow.clamp(shorten * lookahead ,0 , lookahead)


timer_print = 0
keep_moving = True
has_started = False
_task_target_t = 0
_task_target_edge = 0
while keep_moving:
    task_state = task_robot.getState()
    cam_state = camera_robot.getState()
    if not task_state or not cam_state:
        print("Robot ", "task (or both)" if not task_state else "cam", " is not on!")
        break

    # Wait for both robots to say they are waiting to start. (the programs are running)
    if (task_state.output_int_register_0 != 2 or cam_state.output_int_register_0 !=2) and not has_started:
        task_robot.sendWatchdog(1)
        camera_robot.sendWatchdog(1)

        timer_print += 1
        if timer_print % 120 == 1:
            print(" waiting for ", "[task robot]" if task_state.output_int_register_0 != 2 else "", " [camera_robot]" if cam_state.output_int_register_0 != 2 else "")
            print("task config:", [round(q,2) for q in task_state.actual_q])
            print("camera config:", [round(q,2) for q in cam_state.actual_q])
    else:
        # Running!
        task_robot.sendWatchdog(2)
        camera_robot.sendWatchdog(2)

        has_started = True

    # Has started ignores the flags once we are running ( once we do, the robots are no longer "waiting to start" )
    if not has_started:
        continue

    current_task_config = task_state.actual_q
    current_cam_config = cam_state.actual_q

    # Follow camera path
    cam_lookahead_config = PathFollow.getClampedTarget(current_cam_config, PathFollow.getPointFromT(camera_path, _task_target_edge, _task_target_t), TASK_PATH_LOOKAHEAD)
    camera_robot.sendConfig(cam_lookahead_config)
    # Follow task path
    shortened_lookahead = shortenLookahead(TASK_PATH_LOOKAHEAD, current_cam_config, cam_lookahead_config)
    task_lookahead_config, target_edge, target_t = task_follower.getLookaheadData(current_task_config,lookahead_distance=shortened_lookahead)
    task_follower.updateCurrentEdge(current_task_config)
    task_robot.sendConfig(PathFollow.getClampedTarget(current_task_config, task_lookahead_config, TASK_PATH_LOOKAHEAD))

    _task_target_edge = target_edge
    _task_target_t = target_t

    #logger.warning(f"time : {target_t}")
    logger.warning(f"pose: {task_state.actual_TCP_pose}, force: {task_state.actual_TCP_force}")
    print("new line", target_edge, target_t)

