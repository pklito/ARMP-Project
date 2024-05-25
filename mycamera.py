import cv2
import numpy as np
import os
from PIL import Image
from CameraController import *
from Robots import *
import Robots
from time import sleep, time
import numpy as np
import sys
import urx
import math
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

def clamp(value, min, max):
    if value < min:
        return min
    if value > max:
        return max
    return value

# Config
plate_center = (317, 356)
wrist_3_balance = 269.47



# Settings
max_error = 50
kp = -0.08



# Global vars
robot_task = Robots.TaskRobot()
robot_assis = Robots.AssistanceRobot()



if __name__ == '__main__':
    camera = CameraWrapper()
    
    while True:
        
        ret, frame = camera.cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            exit()
            
        position = detect_object(frame)
        if(position is None):
            continue
        
        x1,y1,x2,y2 = position
        ball_center = (int((x1+x2)/2), int((y1+y2)/2))
        
        error = plate_center[0] - ball_center[0]
        print("error: ", plate_center[0] - ball_center[0])

        final = np.deg2rad([34.03, -115.56, -64.3, 0.2, 56.23, wrist_3_balance + kp * error])
        robot_task.move(final, velocity = 1.2, end_early = True)
        
        
        # Get the position of the ball from the frame
        # cv2.imshow('Camera Stream', frame)
        # if cv2.waitKey(1) == ord('q'):
        #     break