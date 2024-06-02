import numpy as np
from UR_Robot_Params import UR5e_PARAMS, UR3e_PARAMS, Transform
import numpy as np
from numpy import linalg
from math import pi, cos, sin, atan2, acos, sqrt, asin


tool_length = 0.135 # [m]

DH_matrix_UR3e = np.matrix([[0, pi / 2.0, 0.15185],
                            [-0.24355, 0, 0],
                            [-0.2132, 0, 0],
                            [0, pi / 2.0, 0.13105],
                            [0, -pi / 2.0, 0.08535],
                            [0, 0, 0.0921 + tool_length]]) # should add tool length?

DH_matrix_UR5e = np.matrix([[0, pi / 2.0, 0.1625],
                            [-0.425, 0, 0],
                            [-0.3922, 0, 0],
                            [0, pi / 2.0, 0.1333],
                            [0, -pi / 2.0, 0.0997],
                            [0, 0, 0.0996 + tool_length]])

DH_matrix_UR10e = np.matrix([[0, pi / 2.0, 0.1807],
                             [-0.6127, 0, 0],
                             [-0.57155, 0, 0],
                             [0, pi / 2.0, 0.17415],
                             [0, -pi / 2.0, 0.11985],
                             [0, 0, 0.11655]])

DH_matrix_UR16e = np.matrix([[0, pi / 2.0, 0.1807],
                             [-0.4784, 0, 0],
                             [-0.36, 0, 0],
                             [0, pi / 2.0, 0.17415],
                             [0, -pi / 2.0, 0.11985],
                             [0, 0, 0.11655]])

DH_matrix_UR3 = np.matrix([[0, pi / 2.0, 0.1519],
                           [-0.24365, 0, 0],
                           [-0.21325, 0, 0],
                           [0, pi / 2.0, 0.11235],
                           [0, -pi / 2.0, 0.08535],
                           [0, 0, 0.0819]])

DH_matrix_UR5 = np.matrix([[0, pi / 2.0, 0.089159],
                           [-0.425, 0, 0],
                           [-0.39225, 0, 0],
                           [0, pi / 2.0, 0.10915],
                           [0, -pi / 2.0, 0.09465],
                           [0, 0, 0.0823]])

DH_matrix_UR10 = np.matrix([[0, pi / 2.0, 0.1273],
                            [-0.612, 0, 0],
                            [-0.5723, 0, 0],
                            [0, pi / 2.0, 0.163941],
                            [0, -pi / 2.0, 0.1157],
                            [0, 0, 0.0922]])


def sphere_collision(p1,p2,r1,r2):
    return np.linalg.norm(np.array(p1)-np.array(p2)) < r1 + r2


def is_in_collision(ur_params, transform, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration 
        """
        global_sphere_coords = transform.conf2sphere_coords(conf)
        # arm - arm collision
        # for i in range(len(ur_params.ur_links) - 2):
        #     for j in range(i + 2, len(ur_params.ur_links)):
        #         spheres = global_sphere_coords[ur_params.ur_links[i]]
        #         other_spheres = global_sphere_coords[ur_params.ur_links[j]]
        #         for s in spheres:
        #             for s2 in other_spheres:
        #                 if sphere_collision(s[0:3], s2[0:3], ur_params.sphere_radius[ur_params.ur_links[i]], ur_params.sphere_radius[ur_params.ur_links[j]]):
        #                     return True
        # arm - obstacle collision - TODO: add this after solving the simple case
        # for joint, spheres in global_sphere_coords.items():
        #     for sphere in spheres:
        #         for obstacle in self.env.obstacles:
        #             if sphere_collision(sphere[0:3], obstacle, self.ur_params.sphere_radius[joint], self.env.radius):
        #                 return True
        # arm - floor collision
        # for joint, spheres in global_sphere_coords.items():
        #     if joint == ur_params.ur_links[0]:
        #         continue
        #     for sphere in spheres:
        #         if sphere[2] < ur_params.sphere_radius[joint]:
        #             return True
        # x - axis limit
        # for joint, spheres in global_sphere_coords.items():
        #     for sphere in spheres:
        #         if sphere[0] + ur_params.sphere_radius[joint] > 0.4:
        #             return True
        return False
    
    
def mat_transform_DH(DH_matrix, n, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    """
    Compute the transformation matrix for the nth joint using Denavit-Hartenberg parameters.

    Parameters:
    DH_matrix (numpy.matrix): The DH parameters matrix for the robot.
    n (int): The index of the joint (1-based index).
    edges (numpy.matrix): The joint angles or displacements.

    Returns:
    numpy.matrix: The transformation matrix for the nth joint.
    """
    n = n - 1
    t_z_theta = np.matrix([[cos(edges[n]), -sin(edges[n]), 0, 0],
                           [sin(edges[n]), cos(edges[n]), 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]], copy=False)
    t_zd = np.matrix(np.identity(4), copy=False)
    t_zd[2, 3] = DH_matrix[n, 2]
    t_xa = np.matrix(np.identity(4), copy=False)
    t_xa[0, 3] = DH_matrix[n, 0]
    t_x_alpha = np.matrix([[1, 0, 0, 0],
                           [0, cos(DH_matrix[n, 1]), -sin(DH_matrix[n, 1]), 0],
                           [0, sin(DH_matrix[n, 1]), cos(DH_matrix[n, 1]), 0],
                           [0, 0, 0, 1]], copy=False)
    transform = t_z_theta * t_zd * t_xa * t_x_alpha
    return transform


def forward_kinematic_solution(DH_matrix, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    """
    Compute the forward kinematic solution for the given joint angles.

    Parameters:
    DH_matrix (numpy.matrix): The DH parameters matrix for the robot.
    edges (numpy.matrix): The joint angles or displacements.

    Returns:
    numpy.matrix: The transformation matrix representing the end-effector's position and orientation.
    """
    
    t01 = mat_transform_DH(DH_matrix, 1, edges)
    t12 = mat_transform_DH(DH_matrix, 2, edges)
    t23 = mat_transform_DH(DH_matrix, 3, edges)
    t34 = mat_transform_DH(DH_matrix, 4, edges)
    t45 = mat_transform_DH(DH_matrix, 5, edges)
    t56 = mat_transform_DH(DH_matrix, 6, edges)
    answer = t01 * t12 * t23 * t34 * t45 * t56
    return answer


def inverse_kinematic_solution(DH_matrix, transform_matrix,):
    """
    Compute the inverse kinematic solution for the given transformation matrix.

    Parameters:
    DH_matrix (numpy.matrix): The DH parameters matrix for the robot.
    transform_matrix (numpy.matrix): The transformation matrix representing the desired end-effector position and orientation.

    Returns:
    numpy.matrix: A matrix containing the possible joint angle solutions.
    """

    theta = np.matrix(np.zeros((6, 8)))
    # theta 1
    T06 = transform_matrix

    P05 = T06 * np.matrix([[0], [0], [-DH_matrix[5, 2]], [1]])
    psi = atan2(P05[1], P05[0])
    phi = acos((DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2]) / sqrt(P05[0] ** 2 + P05[1] ** 2))
    theta[0, 0:4] = psi + phi + pi / 2
    theta[0, 4:8] = psi - phi + pi / 2

    # theta 5
    for i in {0, 4}:
            th5cos = (T06[0, 3] * sin(theta[0, i]) - T06[1, 3] * cos(theta[0, i]) - (
                    DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2])) / DH_matrix[5, 2]
            if 1 >= th5cos >= -1:
                th5 = acos(th5cos)
            else:
                th5 = 0
            theta[4, i:i + 2] = th5
            theta[4, i + 2:i + 4] = -th5
    # theta 6
    for i in {0, 2, 4, 6}:
        T60 = linalg.inv(T06)
        th = atan2((-T60[1, 0] * sin(theta[0, i]) + T60[1, 1] * cos(theta[0, i])),
                   (T60[0, 0] * sin(theta[0, i]) - T60[0, 1] * cos(theta[0, i])))
        theta[5, i:i + 2] = th

    # theta 3
    for i in {0, 2, 4, 6}:
        T01 = mat_transform_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transform_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transform_DH(DH_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01) * T06 * linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])
        costh3 = ((P13[0] ** 2 + P13[1] ** 2 - DH_matrix[1, 0] ** 2 - DH_matrix[2, 0] ** 2) /
                  (2 * DH_matrix[1, 0] * DH_matrix[2, 0]))
        if 1 >= costh3 >= -1:
            th3 = acos(costh3)
        else:
            th3 = 0
        theta[2, i] = th3
        theta[2, i + 1] = -th3

    # theta 2,4
    for i in range(8):
        T01 = mat_transform_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transform_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transform_DH(DH_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01) * T06 * linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])

        theta[1, i] = atan2(-P13[1], -P13[0]) - asin(
            -DH_matrix[2, 0] * sin(theta[2, i]) / sqrt(P13[0] ** 2 + P13[1] ** 2)
        )
        T32 = linalg.inv(mat_transform_DH(DH_matrix, 3, theta[:, i]))
        T21 = linalg.inv(mat_transform_DH(DH_matrix, 2, theta[:, i]))
        T34 = T32 * T21 * T14
        theta[3, i] = atan2(T34[1, 0], T34[0, 0])
    return theta



def get_valid_inverse_solutions(tx, ty, tz, alpha, beta, gamma, DH_matrix, ur_params, transform):
    """
    Compute the inverse kinematic solutions for the given end-effector position.

    Parameters:
    tx (float): The x-coordinate of the end-effector position.
    ty (float): The y-coordinate of the end-effector position.
    tz (float): The z-coordinate of the end-effector position.
    DH_matrix (numpy.matrix): The DH parameters matrix for the robot.

    Returns:
    numpy.ndarray: An array containing the possible joint angle solutions.
    """
    
    transform = np.matrix([[cos(beta) * cos(gamma), sin(alpha) * sin(beta)*cos(gamma) - cos(alpha)*sin(gamma),
                    cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma), tx],
                    [cos(beta)* sin(gamma), sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma),
                     cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma), ty],
                    [-sin(beta), sin(alpha)*cos(beta), cos(alpha)*cos(beta), tz],
                     [0, 0,0,1]])
    
    IKS = inverse_kinematic_solution(DH_matrix, transform)
    candidate_sols = []
    for i in range(IKS.shape[1]):
        candidate_sols.append(IKS[:, i])  
    candidate_sols =  np.array(candidate_sols)
    
    sols = [] 
    for candidate_sol in candidate_sols:
        if is_in_collision(ur_params=ur_params, transform=transform, conf=candidate_sol):
            continue
        for idx, angle in enumerate(candidate_sol):
            if 2*np.pi > angle > np.pi:
                candidate_sol[idx] = -(2*np.pi - angle)
            if -2*np.pi < angle < -np.pi:
                candidate_sol[idx] = -(2*np.pi + angle)
        if np.max(candidate_sol) > np.pi or np.min(candidate_sol) < -np.pi:
            continue  
        sols.append(candidate_sol)
        
    # verify solution:
    valid_sols = []
    for sol in sols:
        transform = forward_kinematic_solution(DH_matrix, sol)
        diff = np.linalg.norm(np.array([transform[0,3],transform[1,3],transform[2,3]])-
                              np.array([tx,ty,tz]))
        if diff < 0.05:
            valid_sols.append(sol)
    valid_sols = np.array(valid_sols)
    return [[c[0] for c in p] for p in valid_sols]

ur_params_ur5e = UR5e_PARAMS(inflation_factor=1)
ur_params_ur3e = UR3e_PARAMS(inflation_factor=1)
transform_ur5e = Transform(ur_params_ur5e)
transform_ur3e = Transform(ur_params_ur3e)

if __name__ == '__main__':    
    # alpha, beta, gamma set the orientation of the end effector [radians]
    alpha = -np.pi
    beta = 0.0
    gamma = 0
    
    # tx, ty, tz set the position of end effector [meter] - in the robot's own frame
    tx = 0.15
    ty = -0.35
    tz = 0.1 # the height to drop a cube for instance
    
    # ur_params_ur5e = UR5e_PARAMS(inflation_factor=1)
    # ur_params_ur3e = UR3e_PARAMS(inflation_factor=1)
    # transform_ur5e = Transform(ur_params_ur5e)
    # transform_ur3e = Transform(ur_params_ur3e)

    try:
        valid_sols = get_valid_inverse_solutions(tx, ty, tz, alpha, beta, gamma, DH_matrix_UR5e, ur_params_ur5e, transform_ur5e)
        print('valid solutions: ', valid_sols)
    except:
        print('No solution found')