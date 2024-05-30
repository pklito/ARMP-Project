import numpy as np
import os
from environment import Environment
from kinematics import UR5e_PARAMS, Transform
from planners import RRT_STAR
from building_blocks import Building_Blocks
from visualizer import Visualize_UR
from building_blocks import get_closest_config
from inverse_kinematics import get_valid_inverse_solutions

def main():
    ur_params = UR5e_PARAMS(inflation_factor=1)
    transform = Transform(ur_params)

    # --------- configurations-------------
    home = np.deg2rad([0, -90, 0, -90, 0,0 ])

    cube1_approach = np.deg2rad([68.8,-68.3, 84.2,-107.1,-90,-18]) #approach without collision
    cube1 = np.deg2rad([69,-63,85, -107.7,-91.3,-18.2]) #actual position of the cube 
    # cube1_goal example - change as you wish #TODO
    # cube1_goal = np.array([-1.20223872,-1.434,-2.3944, -0.883,1.5707,-2.7730])#found by inverse kinematics
    cube1_goal = None

    cube2_approach = np.deg2rad([87.5, -45.5, 47.7, -102, -90.6, 3.3]) #approach without collision
    cube2 = np.deg2rad([86.9, -40.1, 47.7, -102, -90.6, 3.3]) #actual position of the cube 
    #TODO# cube2_goal = #found by inverse kinematics
    cube2_goal = None

    cube3_approach = np.deg2rad([79.7,-46.9,69.7,-105.1,-92.6,-10.1]) #approach without collision
    cube3 = np.deg2rad([80.2, -43.4, 68.9, -107.1,-93.9, -9.4]) #actual position of the cube 
    #TODO# cube3_goal = #found by inverse kinematics
    cube3_goal = None

    # cube4_approach = np.deg2rad([97.6, -38.3,-52.1, -100.8, -90.1, 8.5]) #approach without collision
    cube4_approach = np.deg2rad([97.6, -38.3, 52.1, -100.8, -90.1, 8.5]) 
    cube4 = np.deg2rad([97.6, -34.6, 52.3, -100.8, -90.1, 8.5]) #actual position of the cube 
    #TODO# cube4_goal = #found by inverse kinematics
    cube4_goal = None

    cube5_approach = np.deg2rad([104.6, -85.3,87.7, -90.5,-88.3, 17]) #approach without collision
    cube5 = np.deg2rad([105.1, -86.3, 97.4, -102, -89.8, 19.3]) #actual position of the cube 
    #TODO# cube5_goal = #found by inverse kinematics
    cube5_goal = None
    
    cube6_approach = np.deg2rad([78.3, -61.7, 120.9, -87.6,-12.9, 27.7]) #approach without collision
    cube6 = np.deg2rad([78,-56.6,120.9, -93,-12.7,29.4]) #actual position of the cube 
    #TODO# cube6_goal =#found by inverse kinematics 
    cube6_goal = None
    
    #TODO - change the position of the cubes during the task
    cube1_coords = [-0.10959248574268822, -0.6417732149769166, 0.1390226933317033]
    cube2_coords = [0.08539928976845282, -0.8370930220946053, 0.13813472317717034]
    cube3_coords =  [-0.008445229140271685, -0.7365370847309188, 0.00955541284784159]
    cube4_coords = [0.23647185443765273 ,-0.769747539513382, 0.03971366463235271]
    cube5_coords =[0.26353072323141574 ,-0.4629969534200313, 0.2651034131371637]
    cube6_coords =  [0.26940059242703984, -0.4730222745248458, 0.021688493137064376]
    initial_cubes_coords = [cube1_coords,cube2_coords,cube3_coords,cube4_coords,cube5_coords,cube6_coords]
    
    ##############################################################################################
    fictional_ground = 0.1
    # for Paulo - constructing a: P
    # cube1_final = [-0.106, -0.483, fictional_ground]
    # cube2_final = [-0.106, -0.29, fictional_ground]
    # cube3_final = [-0.106, -0.385, fictional_ground]
    # cube4_final = [-0.176, -0.483, fictional_ground]
    # cube5_final = [-0.166, -0.383, fictional_ground] # always fails with inverse kinematics
    # cube6_final = [-0.22, -0.43, fictional_ground]
    # cube1_final = [-0.25,-0.33, fictional_ground]
    # cube2_final = [-0.25,-0.50, fictional_ground]
    # cube3_final = [-0.25,-0.25, fictional_ground]
    # cube4_final = [-0.175,-0.33, fictional_ground]
    # cube5_final = [-0.175,-0.25, fictional_ground]
    # cube6_final = [-0.25,-0.4, fictional_ground]
    # dir_name = 'P-path'
    ##############################################################################################
    # for Saleh - constructing a: S
    # cube1_final = [-0.08, -0.42, fictional_ground]
    # cube2_final = [-0.15, -0.48, fictional_ground]
    # cube3_final = [-0.23, -0.43, fictional_ground]
    # cube4_final = [-0.14, -0.35, fictional_ground]
    # cube5_final = [-0.2, -0.28, fictional_ground]
    # cube6_final = [-0.12, -0.24, fictional_ground]
    cube1_final = [-0.1,-0.25, fictional_ground]
    cube2_final = [-0.20,-0.25, fictional_ground]
    cube3_final = [-0.13,-0.35, fictional_ground]
    cube4_final = [-0.07,-0.43, fictional_ground]
    cube5_final = [-0.01,-0.50, fictional_ground]
    cube6_final = [-0.15,-0.50, fictional_ground]
    dir_name = 'S-Path-new'
    ##############################################################################################
    final_cubes_coords = [cube1_final,cube2_final,cube3_final,cube4_final,cube5_final,cube6_final]

    # example of how to find coordinates x-y-z from given configuration
    cubex_goal = np.array([-1.20223872,-1.434,-2.3944, -0.883,1.5707,-2.7730])
    manipulator_spheres = transform.conf2sphere_coords(cubex_goal) # forward kinematics
    cube_coords = manipulator_spheres['wrist_3_link'][-1][:3] # take the position of the cube
    print(cube_coords)
    
    cubes = [cube1, cube2, cube3, cube4, cube5, cube6]
    cube_approaches = [cube1_approach, cube2_approach, cube3_approach, cube4_approach, cube5_approach, cube6_approach]
    cube_goals = [cube1_goal, cube2_goal, cube3_goal, cube4_goal, cube5_goal, cube6_goal]
    rrt_start = home
    rrt_goal = cube1_approach
    
    i = 0
    at_cube = False
    plan_list = []
    interim_cube_coords = [c for c in initial_cubes_coords]
    previous_path = []
    while not np.array_equal(rrt_goal, cube_goals[-1]):        
        if at_cube:
            print("current coord: ", final_cubes_coords[i])
            cube_goals[i] = np.array(get_closest_config(get_valid_inverse_solutions(*final_cubes_coords[i],bb=bb), cubes[i]))
            manipulator_spheres = transform.conf2sphere_coords(cube_goals[i]) # forward kinematics
            interim_cube_coords[i] = np.array(manipulator_spheres['wrist_3_link'][-1][:3]) # we moved a cube!
            rrt_start = cube_approaches[i]
            rrt_goal = cube_goals[i]
            add_before = cubes[i]
            add_after = None
            filename = f'cube{i+1}Tocube{i+1}_goal'
            plan_list.append('close') # when moving from cube{i}_goal to cube{i+1}_approaches open the gripper so you can hold next cube
            plan_list.append(filename + '_path.npy')
            i += 1
        else:
            if i == 0:
                rrt_start = home
                rrt_goal = cube_approaches[i]
                filename = f'homeTocube{i+1}'
                add_before = None
            else:
                # rrt_start = ( cube_goals[i - 1] ) if i > 0 else home # could also use the configuration right before cube_goal from previous path as an approximation
                cube_goal_approx = previous_path[len(previous_path)-2] # problematic if the found path has length 2 or less!
                rrt_start = cube_goal_approx if i > 0 else home
                rrt_goal = cube_approaches[i]
                filename = f'{"cube" + str(i) + "_goal" if i > 0 else "home"}Tocube{i+1}_approach'
                add_before = cube_goals[i - 1]
            plan_list.append('open') # when moving from cube{i}_goal to cube{i+1}_approaches open the gripper so you can hold next cube
            plan_list.append(filename + '_path.npy')
            add_after = cubes[i]
        at_cube = not at_cube
        
        env = Environment(env_idx=3, cube_coords=initial_cubes_coords)
        bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05,)
        rrt_star_planner = RRT_STAR(max_step_size=0.5, max_itr=10000, bb=bb)
        initial_cubes_coords = [c for c in interim_cube_coords] # after moving the cubes, check for location changes
        rrt_path = rrt_star_planner.find_path(start_conf=rrt_start, goal_conf=rrt_goal, filename=filename,)
        if len(rrt_path) == 2:
            number_of_configurations_to_check = 9
            new_path = np.linspace(rrt_path[0],rrt_path[1], number_of_configurations_to_check, endpoint=True)
            if not any([bb.is_in_collision(np.array(conf)) for conf in new_path]) == True:
                print("couldn't fix points number issue!")
            else:
                rrt_path = new_path
        if bb.is_in_collision(rrt_start):
            print('start in collision:  ', filename)
        if bb.is_in_collision(rrt_goal): 
            print('goal in collision')
        
        path = []
        if add_before is not None:
            path.append(add_before)
        for conf in rrt_path:
            path.append(conf)
        if add_after is not None:
            path.append(add_after)
        previous_path = np.array([n for n in path])
        print(path)
        np.save(os.path.join(dir_name,filename+'_path'), np.array(path))
    plan_list.append('open') # open the gripper at the end when you reach the goal!
    np.save(os.path.join(dir_name,'plan_list_path'), np.array(plan_list))
    
if __name__ == '__main__':
    main()



# possible errors in S-path:
# cube3_to_cube3_goal_path
# cube3_goal_to_cube4_approach