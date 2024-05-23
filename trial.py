import Robots
from time import sleep, time
import numpy as np
import sys
import urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper



robot_task = Robots.TaskRobot()
robot_assis = Robots.AssistanceRobot()
# rob = urx.Robot('192.168.0.11')
rob = None
while rob is None:
    try:
        rob = urx.Robot('192.168.0.11', use_rt=True)
    except:
        print('Cannot connect to robot. Retrying...')
        sleep(5)
def custom_robot():
    robotiqgrip = Robotiq_Two_Finger_Gripper(robot=rob)
    robotiqgrip.open_gripper()
    robotiqgrip.close_gripper()
    
    
    
def task_robot():
    # robot_task.execute_task()
    # final = np.deg2rad([34.03, -115.56, -64.3, 0.2, 56.23, 269.47])
    # final_path = [robot_task.home, final]
    # robot_task.execute_path(path=final_path)
	 # Instantiate the gripper with the robot connection
    # robotiqgrip = Robotiq_Two_Finger_Gripper(robot=robot_task.robot)
    final = np.deg2rad([34.03, -115.56, -64.3, 0.2, 56.23, 269.47])
    final_path = [robot_task.home, final]
    # robotiqgrip.open_gripper()
    # sleep(1)
    robot_task.execute_path(path=final_path)
    # sleep(1)
    # robotiqgrip.close_gripper()

    # robotiqgrip.open_gripper()
    # robotiqgrip.close_gripper()
    # robot_task.send_program(robotiqgrip.ret_program_to_run())
	
 
 
def assistance_robot():
    my_path=[[0.45210981369018555, -3.539252897302145, 0.9914467970477503, -0.6139599245837708, 1.1233137845993042, 3.199092149734497],
            [-0.08237106004823858, -3.519120832482809, 1.6106651465045374, -1.1102524262717743, 1.054146409034729, 3.9091503620147705],
            [-1.1325977484332483, -1.1808748704246064, -0.9004731774330139, 2.081622524852417, -1.1109607855426233, 0.12972474098205566],
            [-1.1547163168536585, -2.029471536675924, -0.9019871354103088, 2.86674086629834, -1.1632941404925745, 0.14611268043518066],
            [-1.2693751494037073, -2.3390051327147425, -0.4303496181964874, 2.793999357814453, -1.1887553373919886, -0.024665657673970998],
            [-1.2740924994098108, -2.3412076435484828, -0.430588960647583, 2.7770325380512695, -1.2127764860736292, 3.060415506362915],
            [-1.2620652357684534, -1.5316730302623291, -0.4279654026031494, 1.9765478807636718, -1.260444466267721, 3.140753746032715]]
    robot_assis.execute_path(path=my_path)
if __name__ == '__main__':
    task_robot()
    # custom_robot()
    # assistance_robot()