import sys
import os
import time
# Append the parent directory of the current script's directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.Robot.RTDERobot import RTDERobot


if __name__ == '__main__':
    robot = RTDERobot(ROBOT_HOST='192.168.0.12',config_filename="./control_loop_configuration.xml")
    cambot = RTDERobot(ROBOT_HOST='192.168.0.10',config_filename="./control_loop_configuration.xml")
    last_time = 0
    start_time = time.time()
    while True:
        state = robot.getState()
        camstate = cambot.getState()
        if not state or not camstate:
            continue
        if last_time + 4 < time.time():
            print(round(time.time() - start_time, 3))
            print("task: ", [round(q, 3) for q in state.actual_q])
            print("cam: ", [round(q, 3) for q in camstate.actual_q])
            last_time = time.time()

# task:  [-1.254, -0.182, -1.686, -1.134, 2.728, 1.675]
# cam:  [-0.105, -2.148, 1.144, -1.357, -1.609, 0.111]

# task:  [0.1, -0.188, -1.712, -1.178, 1.438, 1.571]
# cam:  [-0.104, -2.13, 1.144, -1.354, -1.406, 0.0]

# task:  [-1.561, -1.446, -2.057, 0.367, 1.571, 1.571]
# cam:  [1.141, -1.252, 0.971, -1.682, -2.777, 0.087]

# task:  [-1.542, -2.684, -0.751, 0.318, 1.579, 1.585]
# cam:  [1.219, -1.186, 0.934, -2.086, -2.372, 0.086]
task:  [-1.561, -1.446, -2.057, 0.367, 1.571, 1.571]
