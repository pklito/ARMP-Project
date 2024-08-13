
from src.MotionUtils.visualizer import Visualize_UR
from src.MotionUtils.UR_Params import UR3e_PARAMS, Transform
from src.MotionUtils.building_blocks import Building_Blocks_UR3e
from src.MotionUtils.environment import Environment
import numpy as np
ur3e_params = UR3e_PARAMS()
transform = Transform(ur3e_params)
env = Environment(0)
bb3 = Building_Blocks_UR3e(transform, ur3e_params, env)
visualize = Visualize_UR(ur3e_params, env, transform, bb3)

visualize.show_conf((0,-np.pi/2, 0, -np.pi/2, 0, 0))

