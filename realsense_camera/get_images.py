from CameraStreamer import CameraStreamer
import cv2
import numpy as np

def try_chessboard_sizes(gray_image, sizes):
    for size in sizes:
        ret, corners = cv2.findChessboardCorners(gray_image, size, None)
        if ret:
            return ret, corners, size
    return False, None, None

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

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
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        ret, corners, size = try_chessboard_sizes(gray, common_sizes)
        if ret:
            print(f"Detected chessboard size: {size}")
            print(corners)
            cv2.imwrite(f"realsense_camera/chessboard/{count}.png", color_image)
            count += 1

            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            # Draw and display the corners
            cv2.drawChessboardCorners(gray, (14,5), corners2, ret)
            cv2.imshow('img', gray)
        else:
            cv2.imshow('img', gray)

        key = cv2.waitKey(1) & 0xFF

 # If found, add object points, image points (after refining them)
        if key == ord('q'):
            cv2.destroyAllWindows()
            loop = False

