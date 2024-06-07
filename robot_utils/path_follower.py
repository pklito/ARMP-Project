from PathFollow import PathFollow, PathFollowStrict
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from RTDERobot import RTDERobot
from constants import *
path = [[0.797, -2.788, -0.017, -0.379, -0.055, -1.566],
       [0.351, -2.031, -0.015, -1.383, 1.233, -1.548],
       [0.291, -1.088, -0.012, -2.096, 1.335, -1.574]]

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
    lookahead_config = pathfollower.getLookaheadConfig(current_config)
    pathfollower.updateCurrentEdge(current_config)
    index = pathfollower.current_edge
    print(lookahead_config - current_config, index)

    robot.sendConfig(lookahead_config)



