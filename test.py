import sys
import urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from time import sleep
import numpy as np

if __name__ == '__main__':
    rob = urx.Robot("192.168.0.11")
    home = [0, -np.pi / 2, 0, -np.pi / 2, 0, 0]
    rob.movej(home, acc=10, vel=0.5)
    print("moving")
