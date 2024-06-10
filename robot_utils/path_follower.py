from PathFollow import PathFollowStrict
import sys
import os
from time import time
import math
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from RTDERobot import RTDERobot
from constants import *
# path = [[0.797, -2.788, -0.017, -0.379, -0.055, -1.566],
#        [0.351, -2.031, -0.015, -1.383, 1.233, -1.548],
#        [0.291, -1.088, -0.012, -2.096, 1.335, -1.574]]

path = [[0.0, -1.571, 0.0, -1.571, 0.0, 0.0],
        [0.033, -2.652, -0.006, -0.44, 1.561, 0.0],
        [0.455, -2.652, -0.023, -0.761, 1.561, 1.0],
        [0.535, -2.855, 1.654, -1.823, 2.547, 0.193],
        [3.563, -2.824, 1.454, -1.623, -1.456, 0.287],
        [0.0, -1.571, 0.0, -1.571, 0.0, 0.0]]

#[-0.222, -2.844, -0.068, -2.154, 4.925, -0.738]

# path = [[0.0, -1.571, 0.0, -1.571, 0.0, 0.0],
#         [0.797, -2.788, -0.017, -0.379, -0.055, -1.566]]

pathfollower = PathFollowStrict(path, TASK_PATH_LOOKAHEAD, TASK_EDGE_CUTOFF)
robot = RTDERobot("192.168.0.11",config_filename="../control_loop_configuration.xml")

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



