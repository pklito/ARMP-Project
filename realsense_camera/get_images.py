from CameraStreamer import CameraStreamer
import cv2
import numpy as np
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParams = cv2.aruco.DetectorParameters()
def get_aruco_corners(color_image):

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParams)
    return ids, corners

def get_object(aruco_ids):
   # milimeters
   SIZE = 52
   TL_DISP = (-(9 + SIZE), 73 + SIZE,0)
   TR_DISP = (8 + SIZE, 73 + SIZE,0)
   BR_DISP = (6.5 + SIZE, -(58 + SIZE),0)
   BL_DISP = (-(19 + SIZE),-( 64 + SIZE),0)
   arucos_obj = dict()
   base = [(-SIZE/2,SIZE/2,0),(SIZE/2,SIZE/2,0),(SIZE/2,-SIZE/2,0),(-SIZE/2,-SIZE/2,0)]
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
    common_sizes = [ (14,5) ]
    loop = True

    while loop:
        color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics = camera.get_frames()
        if color_image is None:
            continue
        ids, corners = get_aruco_corners(color_image)
        if ids is not None:
            object_pts =np.array([ np.array(a) for a in get_object([i[0] for i in ids.tolist()])], dtype=np.float64)
            print(object_pts)
            pixel_pts = corners
            pixel_pts = np.array(pixel_pts, dtype=np.float64)

            # Ensure image_points has shape (N, 1, 2)
            if pixel_pts.shape[1] == 2:
                pixel_pts = pixel_pts.reshape(-1, 1, 2)
            ret, rvec, tvec = cv2.solvePnP(object_pts,pixel_pts,matrix_coeff,dist_coeff)
            print( rvec, tvec)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            loop = False

