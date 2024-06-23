import numpy as np
from environment import Environment
from UR_Params import UR5e_PARAMS, UR3e_PARAMS, Transform
from planner import RRT_STAR
from building_blocks import Building_Blocks, Building_Blocks_UR3e, Building_Blocks_UR5e

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
    print(env_start, env_goal)
    env_start = np.array([1.5183805483757862, -2.810316736312882, 0.6972808296595279, -0.40644833272222725, -0.8925930869302592, 1.1480416945792689])
    env_goal = np.array([2.1551175991809997, 0.6973595069805163, 1.1792304440411918, 3.0061263508998444, 1.3798469479568993, -0.9861298168517258])
    # ---------------------------------------
    filename = 'balancing'
    path = rrt_star_planner.find_path(start_conf=env_start, goal_conf=env_goal, filename=filename)
    print("Finished Planning, stepsize=", stepsize, ", path_length=",len(path), path)
    try:
        np.save(filename + '_path' + '.npy')
    except:
        print('No Path Found')

    ur3e_path = [
        [0, -np.pi/2, 0, -np.pi/2, 0, 0], # home - at this point the robots aren't in sync
        # essentialy, until we pick the plate up we dont care for sync, for simplicity im assuming it's picked up right at the beginning

        np.deg2rad([0, -90, 0, -90, 90, 270]), # 1
        np.deg2rad([0, -102.59, -121.74, 45, 90, 270]), # 2
        np.deg2rad([-21.54, -32.38, -76.96, 31.07, 64.77, 270]), # 3
        np.deg2rad([-33.52, -100.82, -17.93, -61.07, 65.64, 270]), # 4
        np.deg2rad([-100.68, -149.81, -18.76, -11.52, 105.02, 271.85]), # 5
    ]

    ur5e_path = [
        [0, -np.pi/2, 0, -np.pi/2, 0, 0], # home - at this point the robots aren't in sync
        # essentialy, until we pick the plate up we dont care for sync, for simplicity im assuming it's picked up right at the beginning

        np.deg2rad([0, -104.5, 0, -45, -80.81, 0]), # 1
        np.deg2rad([0, -133.26, 23.48, -25.5, -79.45, 0]), # 2
        np.deg2rad([0, -133.26, 23.48, -25.5, -79.45, 0]), # 3
        np.deg2rad([-1.74, -97.97, 23.47, -53.84, -52.75, 0]), # 4
        np.deg2rad([-1.74, -97.97, 23.47, -53.84, -52.75, 0]), # 5
    ]

if __name__ == '__main__':
    main()



