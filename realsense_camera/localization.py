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

## ARUCO HELPER FUNCTIONS ##
def get_aruco_corners(color_image):
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(color_image, ARUCO_DICT, parameters=ARUCO_PARAMS)
    return ids, corners

import time
np.floor(time.time())
VALID_ARUCOS = [a for a in range(21)]

SIZE = 20
VERT_DISP = 10 + SIZE
HOR_DISP = 40 + SIZE
SQUARE_SHAPE = [(-SIZE/2, SIZE/2), (SIZE/2, SIZE/2), (SIZE/2, -SIZE/2), (-SIZE/2, -SIZE/2)]
SHAPE = (3,7)
ARUCO_OBJ = [[((HOR_DISP * (i) + d[0] - HOR_DISP) / 1000., (VERT_DISP*(-j) + d[1] + VERT_DISP*3)/1000., 0) for d in SQUARE_SHAPE]  for j in range(SHAPE[1]) for i in range(SHAPE[0])]
def get_object(aruco_ids):
    # if not all(a in VALID_ARUCOS for a in aruco_ids):
    #     return []
    # milimeters]
    # print(VALID_ARUCOS[0])
    return [p for a in aruco_ids if a in VALID_ARUCOS for p in ARUCO_OBJ[a]]

def get_obj_pxl_points(ids, corners):
    object_pts = np.array(get_object(ids), dtype=np.float32)
    pixel_pts = np.array([c for id, rect in zip(ids, corners) for c in rect[0] if id in VALID_ARUCOS], dtype=np.float32)
    return object_pts, pixel_pts

## Unprojection function ##
def getPixelOnPlane(pixel, rvec, tvec, z_height = BALL_HEIGHT):
    uvcoord = np.array([pixel[0], pixel[1], 1])
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    mat1 = rotation_matrix.T @ CAMERA_MATRIX_INV @ uvcoord
    mat2 = rotation_matrix.T @ tvec

    s = (z_height + mat2[2]) / mat1[2]
    wccoord = rotation_matrix.T @ ((s * CAMERA_MATRIX_INV @ uvcoord) - np.ravel(tvec))
    return wccoord

# if __name__ == "__main__":
#     camera = CameraStreamer()
#     loop = True
#     while loop:
#         color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics = camera.get_frames()
#         if color_image is None:
#             continue
#         gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
#         # cv2.imshow('test', gray)
#         ids, corners = get_aruco_corners(color_image)
#         if ids is not None:
#             object_pts = np.array([a for i in ids for a in get_object([i[0]])], dtype=np.float32)
#             pixel_pts = np.array([c for aruco in corners for c in aruco[0]], dtype=np.float32)
#             if(len(object_pts)!=len(pixel_pts)):
#                 print("Error, sizes", len(object_pts),len(pixel_pts))
#             else:
#                 if pixel_pts.ndim == 3 and pixel_pts.shape[1] == 1 and pixel_pts.shape[2] == 4:
#                     pixel_pts = pixel_pts[:, 0, :]

#                 if object_pts.size == 0 or pixel_pts.size == 0:
#                     continue

#                 ### Get Plane rotation and translation ###
#                 ret, rvec, tvec = cv2.solvePnP(object_pts, pixel_pts, CAMERA_MATRIX, CAMERA_DIST_COEFF)

#                 if(ret):
#                     wcpoint = getPixelOnPlane((WIDTH/2,HEIGHT/2),rvec,tvec)
#                     print([round(a,3) for a in wcpoint])

#                     cv2.drawFrameAxes(color_image, CAMERA_MATRIX, CAMERA_DIST_COEFF, rvec, tvec, 0.026, 2)

#         cv2.imshow("i,", color_image)
#         key = cv2.waitKey(1) & 0xFF
#         if key == ord('q'):
#             cv2.destroyAllWindows()
#             loop = False
