import sys
import urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from time import sleep
import numpy as np

if __name__ == '__main__':
    rob = urx.Robot("192.168.0.11")
    # robotiqgrip = Robotiq_Two_Finger_Gripper(robot=rob)

    # robotiqgrip.gripper_action(255)
    # print("closing")
    # sleep(1)
    home = [0, -np.pi / 2, 0, -np.pi / 2, 0, 0]
    rob.movej(home, acc=10, vel=0.5)
    print("moving")
    # sleep(1)
    # print("opening")
    # robotiqgrip.gripper_action(0)
    # sleep(1)
    # print("closing")
    # robotiqgrip.gripper_action(128)
    # sleep(1)
    # print("done")
    # rob.close()
    # print("false")
    # sys.exit()
