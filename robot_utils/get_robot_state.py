import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from RTDERobot import *

if __name__ == '__main__':
    robot = RTDERobot(ROBOT_HOST='192.168.0.11')
    
    while True:
        state = robot.getState()
        
        print(state.target_q)
        
        robot.sendWatchdog(1)