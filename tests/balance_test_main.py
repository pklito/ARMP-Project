
from time import time
from math import fmod
import numpy as np
from realsense_camera.CameraStreamer import *
from RTDERobot import *
from collections import deque
from simple_pid import PID
import matplotlib.pyplot as plt
debug_plot = []

def my_signal_handler(sig, frame, plot, cam):
    #print("Ctrl-C detected. Stopping camera stream and closing OpenCV windows...")
    print("plot: ", plot)
    cam.stop()
    cv2.destroyAllWindows()
    times = [point[0] for point in debug_plot]
    errors = [point[1] for point in debug_plot]

    # Create the scatter plot
    plt.scatter(times, errors)
    plt.xlabel('Time (s)')
    plt.ylabel('Error')
    plt.title('Time vs Error')
    plt.grid(True)

    # Show the plot
    plt.show()
    sys.exit(0)

from realsense_camera.localization import get_aruco_corners, get_object, getPixelOnPlane, get_obj_pxl_points
from realsense_camera.constants import *

DEBUG = True
def get_ball_position(color_image):
    if color_image is None or color_image.size == 0:
        return None

    positions = detect_ball(color_image)
    if len(positions) == 0:
        if DEBUG:
            print("Ball not detected")
        return None

    ball_center, radius = positions[0]

    ids, corners = get_aruco_corners(color_image)
    if ids is None:
        if DEBUG:
            print("aruco corners not detected")
        return None

    object_pts, pixel_pts = get_obj_pxl_points([a[0] for a in ids.tolist()], corners)

    if(len(object_pts) != len(pixel_pts)):
        if DEBUG:
            print("[Error] obj points, pixel points, ids: ", len(object_pts), len(pixel_pts), ids.tolist())
        return None

    if pixel_pts.ndim == 3 and pixel_pts.shape[1] == 1 and pixel_pts.shape[2] == 4:
        pixel_pts = pixel_pts[:, 0, :]

    if object_pts.size == 0 or pixel_pts.size == 0:
        return None

    ret, rvec, tvec = cv2.solvePnP(object_pts, pixel_pts, CAMERA_MATRIX, CAMERA_DIST_COEFF)

    if ret:
        return getPixelOnPlane((ball_center[0], ball_center[1]),rvec,tvec)
    if DEBUG:
            print("solvePNP failed!")
    return None


print("initializing Robots")
camera = CameraStreamer()
signal.signal(signal.SIGINT, lambda sig, frame: my_signal_handler(sig, frame, debug_plot))
task_robot = RTDERobot("192.168.0.11",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

pid_controller_x = PID(Kp=0.5, Ki=0, Kd=0.5)
pid_controller_x.setpoint = 0

pid_controller_y = PID(Kp=0.3, Ki=0, Kd=0.3)
pid_controller_y.setpoint = 0

plate_center = (0, 0, 0)

print("start!")
keep_moving = True
not_started = True
start_time = time()
initial_pos = [-0.129, -1.059, -1.229, -0.875, 1.716, 1.523]
count = 0
signal.signal(signal.SIGINT, lambda sig, frame: my_signal_handler(sig, frame, debug_plot, camera)) # may not work properly
while keep_moving:
    color_image, _, _, _, _ = camera.get_frames()
    task_state = task_robot.getState()
    cam_state = camera_robot.getState()

    if not task_state.output_int_register_0:
        print("Not started yet", [round(q,2) for q in task_state.target_q])
        continue
    elif not_started:
        not_started = True

    if not task_state or not cam_state:
        break
    task_robot.sendWatchdog(1)
    camera_robot.sendWatchdog(1)

    if color_image is None or color_image.size == 0:
        continue

    # cv2.waitKey(1)
    # cv2.imshow("name", color_image)

    current_task_config = task_state.target_q
    current_cam_config = cam_state.target_q

    ball_position = get_ball_position(color_image)
    if ball_position is None:
        continue
    error = ball_position - plate_center
    debug_plot.append((time()-start_time, error[0]))

    pos = initial_pos.copy()
    pos[5] += pid_controller_x(-error[1])
    pos[3] += pid_controller_y(-error[0])
    print([round(a,3) for a in error])
    task_robot.sendConfig(pos)
