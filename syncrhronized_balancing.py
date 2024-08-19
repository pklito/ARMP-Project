import sys
import os
import numpy as np
from simple_pid import PID
from src.LogGenerator import LoggerGenerator
import time
import datetime
# Append the parent directory of the current script's directory to sys.path
from src.CameraUtils.CameraStreamer import CameraStreamer
from src.CameraUtils.localization import get_aruco_corners, get_object, getPixelOnPlane, get_obj_pxl_points
from src.CameraUtils.cameraConstants.constants import *
from src.CameraUtils.CameraFunctions import *
from src.MotionUtils.motionConstants.constants import *
from src.Robot.RTDERobot import *
import src.MotionUtils.PathFollow as PathFollow

def toView(conf, end = "\n"):
    return [round(a, 2) for a in conf]

# TEMP
def calculateShorterLookahead(lookahead, other_robot_loc, other_robot_target):
    """decrease lookahead based on the other robots position
       [!] DO NOT DECREASE CLAMP!!! otherwise both robots will simply deadlock"""
    other_distance_squared = np.dot(other_robot_loc - other_robot_target, other_robot_loc - other_robot_target)
    shorten = 0.75 / (other_distance_squared + 0.001)
    return PathFollow.clamp(shorten * lookahead ,0 , lookahead)

"""Path follower that maintains the camera runs the same path points as the task robot.
    Requires both robots to use 'rtde_synced_servoj.urp'"""

task_path = [[-1.254, -0.182, -1.686, -1.23, 1.571, 1.571] ,
        [-1.254, -0.182, -1.686, -1.23, 1.571, 1.571] ,
        [0.1, -0.188, -1.712, -1.198, 1.571, 1.571] ,
        [-1.561, -1.446, -2.057, 0.405, 1.571, 1.571] ,
        [-1.542, -2.684, -0.751, 0.337, 1.571, 1.571] ,
        ]

camera_path = [
                [-0.105, -2.148, 1.144, -1.357, -1.609, 0.111],
                [-0.105, -2.148, 1.144, -1.357, -1.609, 0.111],
                [-0.104, -2.13, 1.144, -1.354, -1.406, 0.0],
                [1.141, -1.252, 0.971, -1.682, -2.777, 0.087],
                [1.219, -1.186, 0.934, -2.086, -2.372, 0.086]
]

SLOW_LOOKAHEAD = 0.1
SLOW_EDGE_CUTOFF = 0.05
SLOW_CLAMP = 0.1
curr_time = datetime.datetime.now().strftime("%Y_%m%d_%H%M%S")
logger = LoggerGenerator(logfile=f"logs/synchronized_balancing_{curr_time}.log", consoleLevel=20)
task_follower = PathFollow.PathFollowStrict(task_path, SLOW_LOOKAHEAD, SLOW_EDGE_CUTOFF)
logger.info("Starting Camera")
SHOW_CAMERA = True
camera = CameraStreamer(no_depth=not SHOW_CAMERA)
logger.info("Initialzing task robot")
task_robot = RTDERobot("192.168.0.12",config_filename="control_loop_configuration.xml")
logger.info("Initializing Camera robot")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

pid_controller_x = PID(Kp=0.9, Ki=0, Kd=0.361,output_limits=(-0.4,0.4))
pid_controller_x.setpoint = 0

pid_controller_y = PID(Kp=0.7, Ki=0, Kd=0.2,output_limits=(-0.3,0.3))
pid_controller_y.setpoint = 0

plate_center = (0, 0, 0)

CAMERA_FAILED_IGNORE = 10
CAMERA_FAILED_STOP = 500

logger.warning("Starting the loop.")

timer_print = 0
keep_moving = True
has_started = False
_task_target_t = 0
_task_target_edge = 0
camera_failed_counter = CAMERA_FAILED_STOP + 10
# initial_pos = [-0.129, -1.059, -1.229, -0.875, 1.716, 1.523]

last_offsets = (0,0)
while keep_moving:
    color_image, _, _, depth_colormap, _, Is_image_new = camera.get_frames()
    if( SHOW_CAMERA):
        draw_frame = np.hstack((color_image, depth_colormap))
        cv2.imshow("sync", draw_frame)
        cv2.waitKey(1)
    task_state = task_robot.getState()
    cam_state = camera_robot.getState()

    if not task_state or not cam_state:
        logger.error("Robot " + "[task] " if not task_state else "" + "[camera] " if not cam_state else "" + "is off!")
        continue

    # Wait for both robots to say they are waiting to start. (the programs are running)
    if (task_state.output_int_register_0 != 2 or cam_state.output_int_register_0 !=2) and not has_started:
        task_robot.sendWatchdog(1)
        camera_robot.sendWatchdog(1)

        timer_print += 1
        if timer_print % 120 == 1:
            logger.error("waiting for " + ("[task robot]" if task_state.output_int_register_0 != 2 else "") + (" [camera_robot]" if cam_state.output_int_register_0 != 2 else ""))
            # print("task config:", [round(q,2) for q in task_state.actual_q])
            # print("camera config:", [round(q,2) for q in cam_state.actual_q])
    else:
        # Running!
        task_robot.sendWatchdog(2)
        camera_robot.sendWatchdog(2)

        has_started = True

    # Has started ignores the flags once we are running ( once we do, the robots are no longer "waiting to start" )
    if not has_started:
        continue

    current_task_config = task_state.actual_q
    current_cam_config = cam_state.actual_q

    # We want to read a new ball reading. but skip if the image is old, the ball isn't seen, and many other reasons.
    error = None
    if Is_image_new and color_image is not None and color_image.size != 0:
        ball_position = get_ball_position(color_image,DEBUG=False)
        if ball_position is None:           # no point in reading screen
            camera_failed_counter += 1
            if camera_failed_counter > CAMERA_FAILED_IGNORE:
                logger.error("camera ignored! counter at: " + str(camera_failed_counter))
                error = (0,0)
        else:                               # read new position
            camera_failed_counter = 0
            error = ball_position - plate_center

    # If we didn't get new errors, use the last offsets. else, calculate new ones
    if error is None:
        pass
    else:
        #calculate new PID readings
        if(abs(np.sin(current_task_config[4])) < 0.01):
            last_offsets = (pid_controller_y(0), pid_controller_x(-error[1]))
        else:
            last_offsets = (pid_controller_y(-error[0]/np.sin(current_task_config[4])), pid_controller_x(-error[1]))

    # # Get lookaheads # #
    cam_lookahead_config = PathFollow.getClampedTarget(current_cam_config, PathFollow.getPointFromT(camera_path, _task_target_edge, _task_target_t), SLOW_CLAMP)
    shortened_lookahead = calculateShorterLookahead(SLOW_LOOKAHEAD, current_cam_config, cam_lookahead_config)

    task_lookahead_config, target_edge, target_t = task_follower.getLookaheadData(current_task_config,lookahead_distance=shortened_lookahead)
    _task_target_edge = target_edge
    _task_target_t = target_t

    logger.info("path: " + str( _task_target_edge) + str(_task_target_t))
    # # Follow the paths. # #
    current_ideal_task_config = current_task_config.copy()  # ignore the pid joints.
    current_ideal_task_config[3] = task_lookahead_config[3]
    current_ideal_task_config[5] = task_lookahead_config[5]
    task_follower.updateCurrentEdge(current_ideal_task_config)

    task_config = PathFollow.getClampedTarget(current_ideal_task_config, task_lookahead_config, SLOW_CLAMP).copy() #ideal? maybe real?
    task_config[3] += last_offsets[0]
    task_config[5] += last_offsets[1]
    logger.debug({"error": error, "robot_pos": [round(q,2) for q in task_state.actual_q], "target_pos":[round(q,2) for q in cam_state.actual_q]})
    camera_robot.sendConfig(cam_lookahead_config)

    if camera_failed_counter > CAMERA_FAILED_STOP:
        logger.error("camera failed to find error for a while! halting movement")
        lookahead_again, _, _ = task_follower.getLookaheadData(current_task_config,lookahead_distance=0)
        task_config = PathFollow.getClampedTarget(current_task_config, lookahead_again ,SLOW_CLAMP)
    task_robot.sendConfig(task_config)



