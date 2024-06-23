import cv2
import numpy as np

# ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

BALL_HEIGHT = 0.035

## camera constants ##
WIDTH = 640
HEIGHT = 360
FOV_X = 70.7495
FOV_Y = 43.5411

CAMERA_DIST_COEFF = np.load("dist.npy")
CAMERA_MATRIX = np.load("mtx.npy")
CAMERA_MATRIX_INV = np.linalg.inv(CAMERA_MATRIX)
