from CameraStreamer import CameraStreamer
import cv2

if __name__ == "__main__":
    count = 0
    camera = CameraStreamer()

    loop = True
    while loop:
        color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics = camera.get_frames()
        if color_image is None:
            continue
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow('test', gray)
        key = cv2.waitKey(1) & 0xFF

        ret, corners = cv2.findChessboardCorners(gray, (6,15), None)
        if ret:
            print(corners)
            cv2.imwrite(f"realsense_camera/chessboard/{count}.png",color_image)
            count += 1
        if key == ord('q'):
            cv2.destroyAllWindows()
            loop = False
