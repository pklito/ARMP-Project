import numpy as np
import random

def sphere_collision(p1,p2,r1,r2):
    return np.linalg.norm(np.array(p1)-np.array(p2)) < r1 + r2

def max_angle_difference(conf1, conf2):
    max_difference = 0
    for angle1, angle2 in zip(conf1, conf2):
        # normalize angles to [0, 2 * np.pi] range
        angle1, angle2 = angle1 % (2 * np.pi), angle2 % (2 * np.pi)
        difference = abs(angle1 - angle2)
        # Consider the circular nature of angles
        difference = min(difference % (2 * np.pi), (2 * np.pi - difference) % (2 * np.pi))
        max_difference = max(max_difference, difference)
    return max_difference

def check_sphere_collision(global_sphere_coords_A, global_sphere_coords_B, robotA, robotB):
    """
    Check for collisions between the sphere proxies of two robots.

    Parameters:
    global_sphere_coords_A (dict) : Global coordinates of sphere proxies for robot A.
    global_sphere_coords_B (dict) : Global coordinates of sphere proxies for robot B.
    robotA (Building_Blocks) : The first robot's collision detection system and parameters.
    robotB (Building_Blocks) : The second robot's collision detection system and parameters.

    Returns:
    bool: True if there is a collision between the sphere proxies, False otherwise.
    """
    for linkA, spheresA in global_sphere_coords_A.items():
        for linkB, spheresB in global_sphere_coords_B.items():
            for sphereA in spheresA:
                for sphereB in spheresB:
                    if sphere_collision(sphereA[0:3], sphereB[0:3], robotA.ur_params.sphere_radius[linkA], robotB.ur_params.sphere_radius[linkB]):
                        return True
    return False

def check_two_robot_collision(confA, confB, robotA, robotB):
    """
    Check for any possible collisions between two robots or between the robots and their environment.

    Parameters:
    robotA (Building_Blocks) : The first robot's collision detection system and parameters.
    confA (np.array) : The joint configuration for robot A.
    robotB (Building_Blocks) : The second robot's collision detection system and parameters.
    confB (np.array) : The joint configuration for robot B.

    Returns:
    bool: True if there is a collision either between the robots or between each robot and its environment, False otherwise.
    """
    # Check for self-collision and environmental collisions for each robot using their respective functions
    if robotA.is_in_collision(confA) or robotB.is_in_collision(confB):
        return True

    # Calculate global coordinates of the spheres (collision proxies) for each robot based on their configurations
    global_sphere_coords_A = robotA.transform.conf2sphere_coords(confA)
    global_sphere_coords_B = robotB.transform.conf2sphere_coords(confB)

    # Check for collisions between the sphere proxies of the two robots
    if check_sphere_collision(global_sphere_coords_A, global_sphere_coords_B, robotA, robotB):
        return True

    return False

class Robot:
    def __init__(self, transform, ur_params, env, resolution=0.1, p_bias=0.05):
        self.transform = transform
        self.ur_params = ur_params
        self.env = env
        self.resolution = resolution
        self.p_bias = p_bias
        self.cost_weights = np.array([0.4, 0.3 ,0.2 ,0.1 ,0.07 ,0.05])

class RobotA(Robot):
    def __init__(self, transform, ur_params, env, resolution=0.1, p_bias=0.05):
        super().__init__(transform, ur_params, env, resolution, p_bias)
    
    def is_in_collision(self, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration 
        """
        global_sphere_coords = self.transform.conf2sphere_coords(conf)
        # arm - arm collision
        for i in range(len(self.ur_params.ur_links) - 2):
            for j in range(i + 2, len(self.ur_params.ur_links)):
                spheres = global_sphere_coords[self.ur_params.ur_links[i]]
                other_spheres = global_sphere_coords[self.ur_params.ur_links[j]]
                for s in spheres:
                    for s2 in other_spheres:
                        if sphere_collision(s[0:3], s2[0:3], self.ur_params.sphere_radius[self.ur_params.ur_links[i]], self.ur_params.sphere_radius[self.ur_params.ur_links[j]]):
                            return True
        # arm - obstacle collision
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                for obstacle in self.env.obstacles:
                    if sphere_collision(sphere[0:3], obstacle, self.ur_params.sphere_radius[joint], self.env.radius):
                        return True
        # arm - floor collision
        for joint, spheres in global_sphere_coords.items():
            if joint == self.ur_params.ur_links[0]:
                continue
            for sphere in spheres:
                if sphere[2] < self.ur_params.sphere_radius[joint]:
                    return True
        # x - axis limit
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                if sphere[0] + self.ur_params.sphere_radius[joint] > 0.4:
                    return True
        return False

class RobotB(Robot):
    def __init__(self, transform, ur_params, env, resolution=0.1, p_bias=0.05):
        super().__init__(transform, ur_params, env, resolution, p_bias)

    def is_in_collision(self, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration 
        """
        global_sphere_coords = self.transform.conf2sphere_coords(conf)
        # arm - arm collision
        for i in range(len(self.ur_params.ur_links) - 2):
            for j in range(i + 2, len(self.ur_params.ur_links)):
                spheres = global_sphere_coords[self.ur_params.ur_links[i]]
                other_spheres = global_sphere_coords[self.ur_params.ur_links[j]]
                for s in spheres:
                    for s2 in other_spheres:
                        if sphere_collision(s[0:3], s2[0:3], self.ur_params.sphere_radius[self.ur_params.ur_links[i]], self.ur_params.sphere_radius[self.ur_params.ur_links[j]]):
                            return True
        # arm - obstacle collision
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                for obstacle in self.env.obstacles:
                    if sphere_collision(sphere[0:3], obstacle, self.ur_params.sphere_radius[joint], self.env.radius):
                        return True
        # arm - floor collision
        for joint, spheres in global_sphere_coords.items():
            if joint == self.ur_params.ur_links[0]:
                continue
            for sphere in spheres:
                if sphere[2] < self.ur_params.sphere_radius[joint]:
                    return True
        # x - axis limit
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                if sphere[0] + self.ur_params.sphere_radius[joint] > 0.4:
                    return True
        return False

class Building_Blocks(object):
    def __init__(self, transform, ur_params, env, resolution=0.1, p_bias=0.05):
        self.transform = transform
        self.ur_params = ur_params
        self.env = env
        self.resolution = resolution
        self.p_bias = p_bias
        self.cost_weights = np.array([0.4, 0.3 ,0.2 ,0.1 ,0.07 ,0.05])

    def sample(self, goal_conf) -> np.array:
        conf = []
        if random.random() < self.p_bias:
            return np.array(goal_conf)
        for joint, range in self.ur_params.mechanical_limits.items():
            conf.append(random.uniform(range[0], range[1]))
        return np.array(conf)

    def is_in_collision(self, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration 
        """
        global_sphere_coords = self.transform.conf2sphere_coords(conf)
        # arm - arm collision
        for i in range(len(self.ur_params.ur_links) - 2):
            for j in range(i + 2, len(self.ur_params.ur_links)):
                spheres = global_sphere_coords[self.ur_params.ur_links[i]]
                other_spheres = global_sphere_coords[self.ur_params.ur_links[j]]
                for s in spheres:
                    for s2 in other_spheres:
                        if sphere_collision(s[0:3], s2[0:3], self.ur_params.sphere_radius[self.ur_params.ur_links[i]], self.ur_params.sphere_radius[self.ur_params.ur_links[j]]):
                            return True
        # arm - obstacle collision
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                for obstacle in self.env.obstacles:
                    if sphere_collision(sphere[0:3], obstacle, self.ur_params.sphere_radius[joint], self.env.radius):
                        return True
        # arm - floor collision
        for joint, spheres in global_sphere_coords.items():
            if joint == self.ur_params.ur_links[0]:
                continue
            for sphere in spheres:
                if sphere[2] < self.ur_params.sphere_radius[joint]:
                    return True
        # x - axis limit
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                if sphere[0] + self.ur_params.sphere_radius[joint] > 0.4:
                    return True
        return False

    def local_planner(self, prev_conf, current_conf) -> bool:
        '''check for collisions between two configurations - return True if transition is valid
        @param prev_conf - some configuration
        @param current_conf - current configuration 
        '''
        number_of_configurations_to_check = max(3, int(max_angle_difference(prev_conf, current_conf) / self.resolution))
        return not any([self.is_in_collision(np.array(conf)) for conf in np.linspace(prev_conf, current_conf, number_of_configurations_to_check, endpoint=True)])

    def edge_cost(self, conf1, conf2):
        '''
        Returns the Edge cost- the cost of transition from configuration 1 to configuration 2
        @param conf1 - configuration 1
        @param conf2 - configuration 2
        '''
        return np.dot(self.cost_weights, np.power(conf1 - conf2, 2)) ** 0.5

