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

task_path = [[1.743, -1.458, 2.261, -3.988, -1.571, 1.571] ,
        [1.743, -1.458, 2.261, -3.988, -1.571, 1.571] ,
        [0.676, -1.713, 2.538, -4.01, -1.571, 1.571] ,
        [0.676, -1.872, 2.104, -3.417, -1.571, 1.571] ,
        [0.6, -0.876, -1.509, -0.8, -1.571, 1.571] ,
        [-1.479, -1.444, -1.181, -0.56, -1.571, 1.571] ,
        [-3.038, -1.656, 0.346, -1.875, -1.571, 1.571] ,
        [-3.449, -2.784, 2.257, -2.658, -1.571, 1.571] ,
        ]

camera_path = [
  [0.0, -1.554, -0.0, -0.539, -1.145, -0.0],
  [0.0, -1.554, -0.0, -0.539, -1.145, -0.0],
  [0.043, -1.338, 0.743, -1.646, -1.317, -0.247],
  [0.043, -1.506, 0.696, -1.703, -1.31, -0.42],
  [-0.225, -1.288, 0.22, -1.341, -1.152, -0.414],
  [-0.228, -1.532, 0.22, -1.065, -1.483, -0.414],
  [-0.409, -2.022, 0.673, -0.987, -1.832, -0.415],
  [-0.408, -2.289, 1.152, -1.038, -1.574, -0.139]]

SLOW_LOOKAHEAD = 0.1
SLOW_EDGE_CUTOFF = 0.05
SLOW_CLAMP = 0.1
curr_time = datetime.datetime.now().strftime("%Y_%m%d_%H%M%S")
logger = LoggerGenerator(logfile=f"synchronized_balancing_{curr_time}.log", consoleLevel=20)
task_follower = PathFollow.PathFollowStrict(task_path, SLOW_LOOKAHEAD, SLOW_EDGE_CUTOFF)
logger.info("Starting Camera")
camera = CameraStreamer(no_depth=True)
logger.info("Initialzing task robot")
task_robot = RTDERobot("192.168.0.12",config_filename="control_loop_configuration.xml")
logger.info("Initializing Camera robot")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

pid_controller_x = PID(Kp=0.9, Ki=0, Kd=0.361,output_limits=(-0.4,0.4))
pid_controller_x.setpoint = 0

pid_controller_y = PID(Kp=0.7, Ki=0, Kd=0.2,output_limits=(-0.3,0.3))
pid_controller_y.setpoint = 0

plate_center = (0, 0, 0)

CAMERA_FAILED_MAX = 5

logger.warning("Starting the loop.")

timer_print = 0
keep_moving = True
has_started = False
_task_target_t = 0
_task_target_edge = 0
camera_failed_counter = 100
# initial_pos = [-0.129, -1.059, -1.229, -0.875, 1.716, 1.523]

last_offsets = (0,0)
while keep_moving:
    color_image, _, _, _, _, Is_image_new = camera.get_frames()
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
            logger.error(" waiting for " + "[task robot]" if task_state.output_int_register_0 != 2 else "" + " [camera_robot]" if cam_state.output_int_register_0 != 2 else "")
            # print("task config:", [round(q,2) for q in task_state.target_q])
            # print("camera config:", [round(q,2) for q in cam_state.target_q])
    else:
        # Running!
        task_robot.sendWatchdog(2)
        camera_robot.sendWatchdog(2)

        has_started = True

    # Has started ignores the flags once we are running ( once we do, the robots are no longer "waiting to start" )
    if not has_started:
        continue

    current_task_config = task_state.target_q
    current_cam_config = cam_state.target_q

    # We want to read a new ball reading. but skip if the image is old, the ball isn't seen, and many other reasons.
    error = None
    if Is_image_new and color_image is not None and color_image.size != 0:
        ball_position = get_ball_position(color_image,DEBUG=False)
        if ball_position is None:           # no point in reading screen
            camera_failed_counter += 1
        else:                               # read new position
            camera_failed_counter = 0
            error = ball_position - plate_center

    # If we didn't get new errors, use the last offsets. else, calculate new ones
    if error is None:
        pass
    else:
        #calculate new PID readings
        last_offsets = (pid_controller_y(error[0]), pid_controller_x(-error[1]))

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
    logger.debug({"error": error, "robot_pos": [round(q,2) for q in task_state.target_q], "target_pos":[round(q,2) for q in cam_state.target_q]})
    camera_robot.sendConfig(cam_lookahead_config)

    if camera_failed_counter > CAMERA_FAILED_MAX:
        logger.error("camera failed to find error for a while! halting movement")
        lookahead_again, _, _ = task_follower.getLookaheadData(current_task_config,lookahead_distance=0)
        task_config = PathFollow.getClampedTarget(current_task_config, lookahead_again ,SLOW_CLAMP)
    task_robot.sendConfig(task_config)



