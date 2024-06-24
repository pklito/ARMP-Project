
from time import time
from math import fmod
import numpy as np
import os
import matplotlib.pyplot as plt
from collections import deque
from simple_pid import PID
from urx import Robot
from time import sleep

# Append the parent directory of the current script's directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.CameraUtils.CameraStreamer import *
from src.Robot.RTDERobot import *


def my_signal_handler(sig, frame, cam):
    cam.stop()
    cv2.destroyAllWindows()


print("initializing Robots")
camera = CameraStreamer()
# task_robot = RTDERobot("192.168.0.12",config_filename="control_loop_configuration.xml")
# camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")
task_robot, camera_robot = None, None
while task_robot is None:
    try:
        task_robot = Robot('192.168.0.12', use_rt=True)
    except:
        print('Cannot connect to Task robot. Retrying...')
        sleep(5)

while camera_robot is None:
    try:
        task_robot = Robot('192.168.0.10', use_rt=True)
    except:
        print('Cannot connect to Camera robot. Retrying...')
        sleep(5)
signal.signal(signal.SIGINT, lambda sig, frame: my_signal_handler(sig, frame, camera))

print("initializing PID Controllers")
pid_controller_x = PID(Kp=0.5, Ki=0, Kd=0.5)
pid_controller_x.setpoint = 0

pid_controller_y = PID(Kp=0.3, Ki=0, Kd=0.3)
pid_controller_y.setpoint = 0

print("start!")
ur3e_path = [
        [0, -np.pi/2, 0, -np.pi/2, 0, 0], # home - at this point the robots aren't in sync
        # essentialy, until we pick the plate up we dont care for sync, for simplicity im assuming it's picked up right at the beginning

        np.deg2rad([0, -90, 0, -90, 90, 270]), # 1
        np.deg2rad([0, -102.59, -121.74, 45, 90, 270]), # 2
        np.deg2rad([-21.54, -32.38, -76.96, 31.07, 64.77, 270]), # 3
        np.deg2rad([-33.52, -100.82, -17.93, -61.07, 65.64, 270]), # 4
        np.deg2rad([-100.68, -149.81, -18.76, -11.52, 105.02, 271.85]), # 5

    ]

ur5e_path = [
    [0, -np.pi/2, 0, -np.pi/2, 0, 0], # home - at this point the robots aren't in sync
    # essentialy, until we pick the plate up we dont care for sync, for simplicity im assuming it's picked up right at the beginning

    np.deg2rad([0, -104.5, 0, -45, -80.81, 0]), # 1
    np.deg2rad([0, -133.26, 23.48, -25.5, -79.45, 0]), # 2
    np.deg2rad([0, -133.26, 23.48, -25.5, -79.45, 0]), # 3
    np.deg2rad([-1.74, -97.97, 23.47, -53.84, -52.75, 0]), # 4
    np.deg2rad([-1.74, -97.97, 23.47, -53.84, -52.75, 0]), # 5

]

dist = lambda a, b: np.linalg.norm(np.array(a) - np.array(b))

for index, task_config, cam_config in enumerate(zip(ur3e_path, ur5e_path)):
    if task_config is None or cam_config is None:
        print("Path Execution Completed")
        break

    while (index > 0) and ((dist(task_robot.getj(), task_config) > 0.1) or (dist(camera_robot.getj(), cam_config) > 0.1)):
            pass

    color_image, _, _, _, _ = camera.get_frames()

    if color_image is None or color_image.size == 0:
        continue

    cv2.waitKey(1)
    cv2.imshow("Live Feed", color_image)

    task_robot.movej(task_config, acc=10, vel=0.5)
    camera_robot.movej(task_config, acc=10, vel=0.5)
