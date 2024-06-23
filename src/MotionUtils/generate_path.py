import numpy as np
from environment import Environment
from UR_Params import UR5e_PARAMS, UR3e_PARAMS, Transform
from planner import RRT_STAR
from building_blocks import Building_Blocks_UR3e, Building_Blocks_UR5e

def main():
    ur_params = UR3e_PARAMS(inflation_factor=1)
    env = Environment(env_idx=0) # obstacle free environment
    transform = Transform(ur_params)
    stepsize = 0.1 # I just picked a random value from [0.1, 0.3, 0.5, 0.8, 1.2, 1.7, 2.0]
    bias = 0.05

    bb = Building_Blocks_UR3e(transform=transform, ur_params=ur_params,  env=env, resolution=0.1, p_bias=bias,)
    #visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)
    rrt_star_planner = RRT_STAR(max_step_size=stepsize, max_itr=10000, bb=bb)
    '''
    Note: till now it seems like there's a problem with adjusting the ur parameters to the UR3E robot (especially the min sphere radius dictionary)
    as it results in the robot colliding with it self. this may be unnecessary to adjust if we override the self collision check but that could result in other problems :)
    '''


    # --------- configurations-------------
    env_start = bb.generate_upright_configuration(atol=5e-2) # replace with actual start
    env_goal = bb.generate_upright_configuration(atol=5e-2) # replace with actual goal
    # ---------------------------------------
    filename = 'balancing'
    path = rrt_star_planner.find_path(start_conf=env_start, goal_conf=env_goal, filename=filename)
    print("Finished Planning, stepsize=", stepsize, ", path_length=",len(path), path)
    try:
        np.save(filename + '_path' + '.npy')
    except:
        print('No Path Found')


if __name__ == '__main__':
    main()



