import cv2
import numpy as np
from CameraStreamer import CameraStreamer

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParams = cv2.aruco.DetectorParameters()

WIDTH = 640
#HEIGHT = 480
#FOV_X = 70.7495
HEIGHT = 360
FOV_X = 58.0548
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

if __name__ == "__main__":
    count = 0
    camera = CameraStreamer()
    # common_sizes = [(6, 9), (9, 6), (6, 7), (7, 6), (6, 8), (8, 6), (6, 15), (15, 6), (5,4), (4,5), (5,6), (6,5), (14,5), (5,14)]
    common_sizes = [(14, 5)]
    loop = True

    while loop:
        color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics = camera.get_frames()
        if color_image is None:
            continue
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
                extrinsic_matrix_inv = np.hstack((rotation_matrix.T,-tvec))

                ### Convert pixel to plate world ###
                px = WIDTH/2
                py = HEIGHT/2
                homogeneous_coords = np.array([px-WIDTH/2,py-HEIGHT/2,Z_DIST , 1])

                cam_ball_world = np.ravel(np.reshape(np.dot(extrinsic_matrix_inv,homogeneous_coords),-1))
                cam_focal_world = np.ravel(np.reshape(-tvec,-1))

                ### Calculate plane intersection ###
                a = intersect_ray_with_plane(np.array(cam_focal_world), cam_ball_world - cam_focal_world, np.array([0,0,0]),np.array([0,0,1]))
                print("a", a)
                cv2.drawFrameAxes(color_image, matrix_coeff, dist_coeff, rvec, tvec, 0.026, 2)

        cv2.imshow("i,", color_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            loop = False
