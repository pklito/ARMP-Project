import sys
import os

# Append the parent directory of the current script's directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.MotionUtils.motionConstants.constants import *
from src.Robot.RTDERobot import *
import src.MotionUtils.PathFollow as PathFollow

task_path = [[0.797, -2.788, -0.017, -0.379, -0.055, -1.566],
       [0.351, -2.031, -0.015, -1.383, 1.233, -1.548],
       [0.291, -1.088, -0.012, -2.096, 1.335, -1.574]]

camera_path = [[0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
               [0.0, -1.97, 0.0, -0.91, -0.98, -0.0],
               [0.08, -2.13, 0.97, -1.99, 0.1, -0.0]]
# path = [[0.0, -1.571, 0.0, -1.571, 0.0, 0.0],
#         [0.797, -2.788, -0.017, -0.379, -0.055, -1.566]]

task_follower = PathFollow.PathFollowStrict(task_path, TASK_PATH_LOOKAHEAD, TASK_EDGE_CUTOFF)

task_robot = RTDERobot("192.168.0.12",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

timer_print = 0
keep_moving = True
has_started = False
_task_target_t = 0
_task_target_edge = 0
while keep_moving:
    task_state = task_robot.getState()
    cam_state = camera_robot.getState()
    if not task_state or not cam_state:
        print("Robot ", "task (or both)" if not task_state else "cam", " is not on!")
        break

    task_robot.sendWatchdog(1)
    camera_robot.sendWatchdog(1)

    # Wait for both robots to say they are waiting to start. (the programs are running)
    if (not task_state.output_int_register_0 or not cam_state.output_int_register_0) and not has_started:
        timer_print += 1
        if timer_print % 60 == 1:
            print(" waiting for ", "[task robot]" if not task_state.output_int_register_0 else "", " [camera_robot]" if not cam_state.output_int_register_0 else "")
            print("task config:", [round(q,2) for q in task_state.target_q])
            print("camera config:", [round(q,2) for q in cam_state.target_q])
    else:
        has_started = True

    # Has started ignores the flags once we are running ( once we do, the robots are no longer "waiting to start" )
    if not has_started:
        continue

    current_task_config = task_state.target_q
    current_cam_config = cam_state.target_q

    # Follow camera path
    cam_lookahead_config = PathFollow.getClampedTarget(current_cam_config, PathFollow.getPointFromT(camera_path, _task_target_edge, _task_target_t), TASK_PATH_LOOKAHEAD)
    camera_robot.sendConfig(cam_lookahead_config)

    # Follow task path
    task_lookahead_config, target_edge, target_t = task_follower.getLookaheadData(current_task_config)
    task_follower.updateCurrentEdge(current_task_config)
    task_robot.sendConfig(PathFollow.getClampedTarget(current_task_config, task_lookahead_config, TASK_PATH_LOOKAHEAD))

    _task_target_edge = target_edge
    _task_target_t = target_t

