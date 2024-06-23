import cv2
import numpy as np
import time
from cameraConstants.constants import *


# ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
# ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# ARUCO_PARAMS = cv2.aruco.DetectorParameters()

BALL_HEIGHT = 0.035

## camera constants ##
# WIDTH = 640
# HEIGHT = 360
# FOV_X = 70.7495
# FOV_Y = 43.5411

# CAMERA_DIST_COEFF = np.load("dist.npy")
# CAMERA_MATRIX = np.load("mtx.npy")
# CAMERA_MATRIX_INV = np.linalg.inv(CAMERA_MATRIX)

## ARUCO HELPER FUNCTIONS ##
def get_aruco_corners(color_image):
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(color_image, ARUCO_DICT, parameters=ARUCO_PARAMS)
    return ids, corners

np.floor(time.time())
VALID_ARUCOS = [a for a in range(21)]

SIZE = 20
VERT_DISP = 10 + SIZE
HOR_DISP = 40 + SIZE
SQUARE_SHAPE = [(-SIZE/2, SIZE/2), (SIZE/2, SIZE/2), (SIZE/2, -SIZE/2), (-SIZE/2, -SIZE/2)]
SHAPE = (3,7)
ARUCO_OBJ = [[((HOR_DISP * (i) + d[0] - HOR_DISP) / 1000., (VERT_DISP*(-j) + d[1] + VERT_DISP*3)/1000., 0) for d in SQUARE_SHAPE]  for j in range(SHAPE[1]) for i in range(SHAPE[0])]

def get_object(aruco_ids):
    return [p for a in aruco_ids if a in VALID_ARUCOS for p in ARUCO_OBJ[a]]

MAX_ARUCOS = 100
def get_obj_pxl_points(ids, corners):
    object_pts = np.array(get_object(ids), dtype=np.float32)
    pixel_pts = np.array([c for id, rect in zip(ids, corners) for c in rect[0] if id in VALID_ARUCOS], dtype=np.float32)
    return object_pts[:min(len(object_pts), MAX_ARUCOS)], pixel_pts[:min(len(pixel_pts), MAX_ARUCOS)]

## Unprojection function ##
def getPixelOnPlane(pixel, rvec, tvec, z_height = BALL_HEIGHT):
    uvcoord = np.array([pixel[0], pixel[1], 1])
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    mat1 = rotation_matrix.T @ CAMERA_MATRIX_INV @ uvcoord
    mat2 = rotation_matrix.T @ tvec

    s = (z_height + mat2[2]) / mat1[2]
    wccoord = rotation_matrix.T @ ((s * CAMERA_MATRIX_INV @ uvcoord) - np.ravel(tvec))
    return wccoord
