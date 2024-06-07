import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from RTDERobot import *

if __name__ == '__main__':
    robot = RTDERobot(ROBOT_HOST='192.168.0.11',config_filename="../control_loop_configuration.xml")

    while True:
        state = robot.getState()

        print([round(q, 3) for q in state.target_q])

        robot.sendWatchdog(1)
