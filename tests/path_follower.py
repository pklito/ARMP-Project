import sys
import os
import math
import numpy as np
from time import time

# Append the parent directory of the current script's directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.MotionUtils.PathFollow import PathFollowStrict
from src.MotionUtils.motionConstants.constants import *
from src.Robot.RTDERobot import RTDERobot


# path = [[0.797, -2.788, -0.017, -0.379, -0.055, -1.566],
#        [0.351, -2.031, -0.015, -1.383, 1.233, -1.548],
#        [0.291, -1.088, -0.012, -2.096, 1.335, -1.574]]

# path = [[0.0, -1.571, 0.0, -1.571, 0.0, 0.0],
#         [0.033, -2.652, -0.006, -0.44, 1.561, 0.0],
#         [0.455, -2.652, -0.023, -0.761, 1.561, 1.0],
#         [0.535, -2.855, 1.654, -1.823, 2.547, 0.193],
#         [3.563, -2.824, 1.454, -1.623, -1.456, 0.287],
#         [0.0, -1.571, 0.0, -1.571, 0.0, 0.0]]

#[-0.222, -2.844, -0.068, -2.154, 4.925, -0.738]

# path = [[0.0, -1.571, 0.0, -1.571, 0.0, 0.0],
#         [0.797, -2.788, -0.017, -0.379, -0.055, -1.566]]

initial_pos = [0.12, -2.47, 0.018, -0.65, 1.456, -1.608]
goal_position = np.deg2rad([40.88, -12.77, -28.43, -142.90, -34.73, 273.11])
path = np.linspace(initial_pos, goal_position,1000)
for conf in path:
    # these values will be dictated by the error!
    conf[3] = initial_pos[3]
    conf[5] = initial_pos[5]

pathfollower = PathFollowStrict(path, TASK_PATH_LOOKAHEAD, TASK_EDGE_CUTOFF)
robot = RTDERobot("192.168.0.11",config_filename="../Robot/control_loop_configuration.xml")

keep_moving = True
while keep_moving:
    state = robot.getState()
    if not state:
        break
    robot.sendWatchdog(1)

    current_config = state.target_q
    lookahead_config = pathfollower.getLookaheadConfig(current_config, 0.15 + 0.15*math.sin(time()*2))
    pathfollower.updateCurrentEdge(current_config)
    index = pathfollower.current_edge
    print(lookahead_config - current_config, index)

    robot.sendConfig(lookahead_config)



