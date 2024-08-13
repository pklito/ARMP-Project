
from src.MotionUtils.visualizer import Visualize_UR
from src.MotionUtils.UR_Params import UR3e_PARAMS, Transform
from src.MotionUtils.building_blocks import Building_Blocks_UR3e
from src.MotionUtils.environment import Environment
import numpy as np
from time import sleep
ur3e_params = UR3e_PARAMS()
transform = Transform(ur3e_params)
env = Environment(0)
bb3 = Building_Blocks_UR3e(transform, ur3e_params, env)
visualize = Visualize_UR(ur3e_params, env, transform, bb3)

visualize.show_conf((0,-np.pi/2, 0, -np.pi/2, 0, 0))
task_path = [
    [0.3, -1.059, -1.229, -0.81, 1.571, 1.571] ,
        [0.3, -1.059, -1.229, -0.81, 1.571, 1.571] ,
        [-0.406, -1.349, -1.227, -0.522, 1.571, 1.571] ,
        [-0.619, -2.458, -1.248, 0.608, 1.571, 1.571] ,
        [0.237, -1.844, -1.927, 0.673, 1.571, 1.571] ,
        [0.3, -1.059, -1.229, -0.81, 1.571, 1.571] ,
        ]

# for conf in task_path:
#     visualize.show_conf(conf)
#     sleep(1)
visualize.show_path(task_path, sleep_time=0.1)
