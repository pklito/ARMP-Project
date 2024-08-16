import cv2
import numpy as np
from src.CameraUtils.cameraConstants.constants import *

cap = cv2.VideoCapture(0)
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
text_img_shape = (int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))

def easeInOutSine(x):
    return -(np.cos(np.pi * x) - 1) / 2

def draw_filling_aruco_square(image, aruco_corners, percent, color = (0,200,0), easing = easeInOutSine):
    if percent > 1:
        percent = 1
    if percent < 0:
        percent = 0
    text_img = np.zeros(image.shape[:2], dtype=np.uint8)
    leftmost_corner = min(aruco_corners,key=lambda x: x[0])
    rightmost_corner = max(aruco_corners,key=lambda x: x[0])
    topmost_corner = min(aruco_corners,key=lambda x: x[1])
    bottom_corner = max(aruco_corners,key=lambda x: x[1])

    draw_until = int(leftmost_corner[0] + easing(percent)*(rightmost_corner[0]-leftmost_corner[0]))

    pts = corner.reshape(4, 2)
    pts = pts.astype(np.int32)
    cv2.fillPoly(text_img, [pts], color=255)
    cv2.fillConvexPoly(text_img, np.array([(draw_until,topmost_corner[1]), (rightmost_corner[0],topmost_corner[1]), (rightmost_corner[0],bottom_corner[1]), (draw_until,bottom_corner[1])],dtype=np.int32),0)

    if percent >= 1:
        image[text_img == 255] //= np.array((10,10,10),dtype=np.uint8)
        image[text_img == 255] += np.array((0,200,0),dtype=np.uint8)

    else:
        image[text_img == 255] //= np.array((4,2,4),dtype=np.uint8)
        image[text_img == 255] += np.array((0,128*easing(percent),0),dtype=np.uint8)
    cv2.polylines(image, [pts], isClosed=True, color=(50,255,50),thickness=2)



def draw_text_on_aruco(image, aruco_corners, text, color = (255,255,255), font = cv2.FONT_HERSHEY_SIMPLEX, font_size = 3, font_thickness = 3):
    # Create text image with the text we want.
    text_img = np.zeros(image.shape[:2], dtype=np.uint8)
    center = cv2.getTextSize(text, font, font_size, font_thickness)[0]
    cv2.putText(text_img, text, (text_img_shape[0]//2-center[0]//2,text_img.shape[1]//2 + center[1]//2), font, font_size, 255, font_thickness, cv2.LINE_AA)

    # Modify aruco coordinates.
    pts_normalized = np.array([p - (aruco_corners[3]-aruco_corners[0])/1.8 for p in aruco_corners],dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(np.array([(0,0),(text_img_shape[0], 0),(text_img_shape[0],text_img_shape[1]),(0,text_img_shape[1])]).astype(np.float32), pts_normalized)
    tilted_text_img = cv2.warpPerspective(text_img, matrix, (text_img_shape[1], text_img_shape[0]))

    mask = tilted_text_img > 0
    image[mask] = color
# Put text on the image

def draw_arrow_in_aruco(color_image, aruco_corners, color = (0,0,0), thickness=5, tipLength=0.1):
    aruco_center = sum(aruco_corners)/4
    offset = (aruco_center - (np.array(color_image.shape[:2])[::-1]/2))*1.5
    cv2.arrowedLine(color_image, (aruco_center - offset/30).astype(np.int32),(aruco_center + offset/30).astype(np.int32), color=color, thickness=2, tipLength=0.3)

time = 0

while True:
    ret, color_image = cap.read()
    if not ret:
        break

    time += 0.01
    if time > 1.5:
        time = 0
    corners, ids, _ = cv2.aruco.detectMarkers(color_image, ARUCO_DICT, parameters=ARUCO_PARAMS)
    if ids is not None:
        for id, corner in zip(ids,corners):
            pts = corner.reshape(4, 2)
            pts = pts.astype(np.int32)
            if id == 100:
                aruco_center = sum(pts)/4

                draw_text_on_aruco(color_image, pts, "Move", color = (000,30,000))
                cv2.polylines(color_image, [np.array([0.8 * p + 0.2 * aruco_center for p in pts],dtype=np.int32)], isClosed=True, color=(0, 255, 0),thickness=1)
                cv2.polylines(color_image, [pts], isClosed=True, color=(0, 255, 0),thickness=2)

                draw_arrow_in_aruco(color_image, pts)
            elif id == 101:
                draw_text_on_aruco(color_image, pts, "Store", color=(255,100,0))
                draw_filling_aruco_square(color_image, pts, time)
    cv2.imshow("frame", color_image)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:  # 27 is the ASCII code for ESC
        break

cap.release()
cv2.destroyAllWindows()

