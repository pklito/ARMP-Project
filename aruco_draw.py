import cv2
import numpy as np
from src.CameraUtils.cameraConstants.constants import *

cap = cv2.VideoCapture(0)
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
text_img_shape = (int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
text_img = np.zeros((text_img_shape[0],text_img_shape[1]), dtype=np.uint8)
# Put text on the image
font = cv2.FONT_HERSHEY_SIMPLEX
font_size = 3
font_thickness = 3
text = 'MOVE'
center = cv2.getTextSize(text, font, font_size, font_thickness)[0]
cv2.putText(text_img, text, (text_img_shape[0]//2-center[0]//2,text_img.shape[1]//2 + center[1]//2), font, font_size, 255, font_thickness, cv2.LINE_AA)
while True:
    ret, color_image = cap.read()
    if not ret:
        break

    corners, ids, _ = cv2.aruco.detectMarkers(color_image, ARUCO_DICT, parameters=ARUCO_PARAMS)
    if ids is not None:
        for corner in corners:
            pts = corner.reshape(4, 2)
            pts = pts.astype(np.int32)
            pts_normalized = np.array([p - (pts[3]-pts[0]+(0,10))/2 for p in pts],dtype=np.float32)
            matrix = cv2.getPerspectiveTransform(np.array([(0,0),(text_img_shape[0], 0),(text_img_shape[0],text_img_shape[1]),(0,text_img_shape[1])]).astype(np.float32), pts_normalized)
            tilted_text_img = cv2.warpPerspective(text_img, matrix, (text_img_shape[1], text_img_shape[0]))
            cv2.fillPoly(color_image, [pts], color=(0, 255, 0))



            # Get the dimensions of the foreground image
            f_height, f_width = tilted_text_img.shape[:2]
            mask = tilted_text_img > 0
            print(mask.shape)
            # Overlay the foreground image on the background image
            print(color_image.shape)
            color_image[mask] = (255,255,255)


    cv2.imshow("frame", color_image)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:  # 27 is the ASCII code for ESC
        break

cap.release()
cv2.destroyAllWindows()

