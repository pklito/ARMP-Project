
from time import time
from math import fmod
import numpy as np
from realsense_camera.CameraStreamer import *
from RTDERobot import *
import robot_utils.kinematicsUtils as fk

print("here")
camera = CameraStreamer()
task_robot = RTDERobot("192.168.0.11",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

def get_world_position_from_buffers(pixel_x, pixel_y, depth_frame, depth_intrinsic):
    depth = depth_frame.get_distance(pixel_x, pixel_y)

    # Convert pixel coordinates to world coordinates (from the Camera's Perspective!)
    ball_position = rs.rs2_deproject_pixel_to_point(depth_intrinsic, [pixel_x, pixel_y], depth)
    return np.array(ball_position)

def get_position():
    color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics = camera.get_frames()
    if color_image is None or depth_image is None:
        # print("None Frames")
        return None  # Return None to indicate an error state

    if color_image.size == 0 or depth_image.size == 0:
        print("Can't receive frame (stream end?).")
        return None  # Return None to indicate an error state

    positions = detect_object(color_image)
    if len(positions) == 0:
        print('No object detected!')
        return None

    x1, y1, x2, y2 = positions[0]
    ball_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
    depth_value = camera.depth_frame.get_distance(ball_center[0], ball_center[1])
    distance_meters = depth_value * 1000  # Convert to millimeters
    camera_world_coords = camera.get_world_position_from_camera(ball_center[0], ball_center[1])

    color = (0, 255, 0)  # Green for ball
    cv2.rectangle((color_image), (x1, y1), (x2, y2), color, 2)
    cv2.circle((color_image), ball_center, 3 ,color,2)
    cv2.putText((color_image), f'Ball Distance: {distance_meters:.2f} mm,', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.putText((color_image), f'Ball Center: ({ball_center[0]}, {ball_center[1]}),', (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.putText((color_image), f'Ball World Coordinates: {camera_world_coords}', (x1, y1 - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return get_world_position_from_buffers(ball_center[0],ball_center[1], depth_frame,depth_intrinsics)

last_pos = [0,0,0,0]
i = 0
print("start!")
keep_moving = True
while keep_moving:
    task_state = task_robot.getState()
    cam_state = camera_robot.getState()
    if not task_state or not cam_state:
        break
    task_robot.sendWatchdog(1)
    camera_robot.sendWatchdog(1)

    current_task_config = task_state.target_q
    current_cam_config = cam_state.target_q

    ball_position = get_position()
    if ball_position is None:
        continue
    # [-0.0148 -0.0261 -0.0952  0.    ] --> [ 0.0093 -0.0265 -0.1055  0.    ] ---> [0.5577 0.2865 0.3697 0.    ] --> [ 0.0113  0.0458 -0.0936  0.    ]
    i += 1
    camera.stream_frames()

    if i > 50:
        plate_pos = fk.ur3e_effector_to_home(current_task_config,fk.plate_from_ee([0,0,0,1]))
        ball_pos = fk.ur5e_effector_to_home(current_cam_config, fk.camera_from_ee(ball_position))
        print("ball_pos: ", ball_pos, ball_pos - last_pos)
        last_pos = ball_pos
        # camera.stream_frames()
        # print("difference: ", plate_pos - ball_pos)
        i = 0
