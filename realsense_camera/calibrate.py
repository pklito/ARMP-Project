import numpy as np
import cv2
import os
import glob

if __name__ == '__main__':
   criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

   # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
   objp = np.zeros((14*5,3), np.float32)
   objp[:,:2] = np.mgrid[0:14,0:5].T.reshape(-1,2)

   # Arrays to store object points and image points from all the images.
   objpoints = [] # 3d point in real world space
   imgpoints = [] # 2d points in image plane.

   images = glob.glob('./realsense_camera/chessboard/**1.png')

   gray = None
   for fname in images:
      img = cv2.imread(fname)
      gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

   # Find the chess board corners
      ret, corners = cv2.findChessboardCorners(gray, (14,5), None)

      if ret == True:
         objpoints.append(objp)

         corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
         imgpoints.append(corners2)

         cv2.drawChessboardCorners(img, (14,5), corners2, ret)
         print("done one")

   print("calibarting")
   ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
   print(mtx, dist)
   np.save('mtx.npy',mtx)
   np.save('dist.npy',dist)

   print("fixing")
   img = cv2.imread('./realsense_camera/chessboard/0.png')
   h, w = img.shape[:2]
   newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

   print('undistorting')
   dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

   print("cropping")
   x, y, w, h = roi
   dst = dst[y:y+h, x:x+w]
   cv2.imwrite('0result.png', dst)
   cv2.imshow("test", dst)

   if len(objpoints) > 0 and len(imgpoints) > 0:
      camera_matrix = np.array(mtx, dtype=np.float64)
      dist_coefs = np.array(dist, dtype=np.float64)
      object_points = objpoints[-1]
      image_points = imgpoints[-1]

      object_points = np.array(object_points, dtype=np.float64)
      image_points = np.array(image_points, dtype=np.float64)

      # Ensure image_points has shape (N, 1, 2)
      if image_points.shape[1] == 2:
         image_points = image_points.reshape(-1, 1, 2)

      ret, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coefs)
      if ret:
         rotation_matrix, _ = cv2.Rodrigues(rvec)
         print("Rotation Vector:\n", rvec)
         print("Translation Vector:\n", tvec)
         print("Rotation Matrix:\n", rotation_matrix)
         np.save('rvec.npy',rvec)
         np.save('tvec.npy',tvec)
         np.save('rotation_matrix.npy', rotation_matrix)
      else:
         print("Not enough points for solvePnP.")

   cv2.waitKey(500)
   cv2.destroyAllWindows()
