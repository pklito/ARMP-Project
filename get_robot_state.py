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
            print("task: ", [round(q, 3) for q in state.target_q])
            print("cam: ", [round(q, 3) for q in camstate.target_q])
            last_time = time.time()
