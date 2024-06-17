import cv2
import numpy as np
from platformdirs import user_videos_dir
from CameraStreamer import CameraStreamer

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParams = cv2.aruco.DetectorParameters()

WIDTH = 640
HEIGHT = 360
FOV_X = 70.7495
FOV_Y = 43.5411

Z_DIST = 0.5*WIDTH/np.tan(np.deg2rad(FOV_X/2))

import numpy as np

def intersect_ray_with_plane(ray_origin, ray_direction, plane_point, plane_normal):
    """
    Calculate the intersection of a ray with a plane.

    :param ray_origin: Origin of the ray (3D point) as a numpy array.
    :param ray_direction: Direction of the ray (3D vector) as a numpy array.
    :param plane_point: A point on the plane (3D point) as a numpy array.
    :param plane_normal: Normal vector of the plane (3D vector) as a numpy array.
    :return: Intersection point (3D point) as a numpy array, or None if no intersection.
    """
    # Ensure the direction vector is normalizedm
    ray_direction = ray_direction / np.linalg.norm(ray_direction)
    # Calculate the dot product of ray direction and plane normal
    denom = np.dot(ray_direction, plane_normal)

    # Check if the ray is parallel to the plane
    if np.abs(denom) < 1e-6:
        return None  # No intersection, the ray is parallel to the plane

    # Calculate the parameter t for the ray equation
    t = np.dot(plane_point - ray_origin, plane_normal) / denom

    # Calculate the intersection point
    intersection_point = ray_origin + t * ray_direction

    return intersection_point



def get_aruco_corners(color_image):
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParams)
    # if ids is None:
    #     return None, None
    # l = zip(ids,corners)
    # for a,b in l:
    #     if a[0] == 600:
    #         return [a], [b]
    return ids, corners

def get_object(aruco_ids):
    if not all(a in [200,300,400,500,600] for a in aruco_ids):
        return []
    # milimeters
    SIZE = 52
    TL_DISP = (-(73 + SIZE), -(9 + SIZE), 0)
    TR_DISP = (-(73 + SIZE), 8 + SIZE , 0)
    BR_DISP = ((58 + SIZE), (6.5 + SIZE), 0)
    BL_DISP = ( (64 + SIZE), -(19 + SIZE), 0)
    arucos_obj = dict()
    base = [(-SIZE/2, SIZE/2, 0), (SIZE/2, SIZE/2, 0), (SIZE/2, -SIZE/2, 0), (-SIZE/2, -SIZE/2, 0)]
    arucos_obj[200] = [[c/1000. for c in points] for points in base]
    arucos_obj[300] = [[(coord + displacement)/1000. for coord, displacement in zip(points, TL_DISP)] for points in base]
    arucos_obj[500] = [[(coord + displacement)/1000. for coord, displacement in zip(points, TR_DISP)] for points in base]
    arucos_obj[400] = [[(coord + displacement)/1000. for coord, displacement in zip(points, BR_DISP)] for points in base]
    arucos_obj[600] = [[(coord + displacement)/1000. for coord, displacement in zip(points, BL_DISP)] for points in base]

    return [a for i in aruco_ids for a in arucos_obj[i]]

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

dist_coeff = np.load("dist.npy")
matrix_coeff = np.load("mtx.npy")
matrix_coeff_inv = np.linalg.inv(matrix_coeff)
if __name__ == "__main__":
    camera = CameraStreamer()
    loop = True
    while loop:
        color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics = camera.get_frames()
        if color_image is None:
            continue
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('test', gray)
        ids, corners = get_aruco_corners(color_image)
        if ids is not None:
            object_pts = np.array([a for i in ids for a in get_object([i[0]])], dtype=np.float32)
            pixel_pts = np.array([c for aruco in corners for c in aruco[0]], dtype=np.float32)
            if(len(object_pts)!=len(pixel_pts)):
                print("Error, sizes", len(object_pts),len(pixel_pts))
            else:
                if pixel_pts.ndim == 3 and pixel_pts.shape[1] == 1 and pixel_pts.shape[2] == 4:
                    pixel_pts = pixel_pts[:, 0, :]

                if object_pts.size == 0 or pixel_pts.size == 0:
                    continue

                ### Get Plane rotation and translation ###
                ret, rvec, tvec = cv2.solvePnP(object_pts, pixel_pts, matrix_coeff, dist_coeff)
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                uvcoord = np.array([WIDTH/2, HEIGHT/2, 1])
                mat1 = np.ravel(np.dot(rotation_matrix.T, np.ravel(np.dot(matrix_coeff_inv,uvcoord))))
                print(mat1, mat1.shape)


                cv2.drawFrameAxes(color_image, matrix_coeff, dist_coeff, rvec, tvec, 0.026, 2)

        cv2.imshow("i,", color_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            loop = False
