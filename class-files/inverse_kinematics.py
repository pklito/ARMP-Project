import numpy as np
from environment import Environment
from kinematics import UR5e_PARAMS, UR3e_PARAMS, Transform
from building_blocks import Building_Blocks
from visualizer import Visualize_UR
import numpy as np
from numpy import linalg
from math import pi, cos, sin, atan2, acos, sqrt, asin


tool_length = 0.135 # [m]

DH_matrix_UR3e = np.matrix([[0, pi / 2.0, 0.15185],
                            [-0.24355, 0, 0],
                            [-0.2132, 0, 0],
                            [0, pi / 2.0, 0.13105],
                            [0, -pi / 2.0, 0.08535],
                            [0, 0, 0.0921]]) # should add tool length?

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


def mat_transform_DH(DH_matrix, n, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
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
    t01 = mat_transform_DH(DH_matrix, 1, edges)
    t12 = mat_transform_DH(DH_matrix, 2, edges)
    t23 = mat_transform_DH(DH_matrix, 3, edges)
    t34 = mat_transform_DH(DH_matrix, 4, edges)
    t45 = mat_transform_DH(DH_matrix, 5, edges)
    t56 = mat_transform_DH(DH_matrix, 6, edges)
    answer = t01 * t12 * t23 * t34 * t45 * t56
    return answer


def inverse_kinematic_solution(DH_matrix, transform_matrix,):

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
        # if sin(theta[4, i]) == 0:
        #     theta[5, i:i + 1] = 0 # any angle
        #     break
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



def inverse_kinematics_solutions_endpos(tx, ty, tz, DH_matrix):
    # alpha, beta, gamma set the orientation of the end effector [radians]
    alpha = -np.pi
    beta = 0.0
    gamma = 0
    
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
    return np.array(candidate_sols)



def get_valid_inverse_solutions(tx,ty,tz,bb, DH_matrix):
    candidate_sols = inverse_kinematics_solutions_endpos(tx, ty, tz)
    sols = [] 
    for candidate_sol in candidate_sols:
        if bb.is_in_collision(candidate_sol):
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
    final_sol = []
    for sol in sols:
        transform = forward_kinematic_solution(DH_matrix, sol)
        diff = np.linalg.norm(np.array([transform[0,3],transform[1,3],transform[2,3]])-
                              np.array([tx,ty,tz]))
        if diff < 0.05:
            final_sol.append(sol)
    final_sol = np.array(final_sol)
    return [[c[0] for c in p] for p in final_sol]

if __name__ == '__main__':    
    # alpha, beta, gamma set the orientation of the end effector [radians]
    alpha = -np.pi
    beta = 0.0
    gamma = 0
    
    # tx, ty, tz set the position of end effector [meter]
    tx = 0.15
    ty = -0.35
    tz = 0.1 # the height to drop a cube for instance
    
    # env_idx sets the corresponding environment with obstacles
    env_idx = 3
    # ------------------------------
    
    DH_matrix, ur_params = DH_matrix_UR5e, UR5e_PARAMS(inflation_factor=1)
    # DH_matrix, ur_params = DH_matrix_UR3e, UR3e_PARAMS(inflation_factor=1)

    transform = Transform(ur_params)
    env = Environment(env_idx=env_idx)
    bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.05)
    # visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)

    try:
        cube = [-0.08, -0.42, 0.1] #example
        valid_sols = get_valid_inverse_solutions(*cube,bb=bb, DH_matrix=DH_matrix)
        print('valid solutions: ', valid_sols)
        
        # visualizer.show_path(final_sol)
        # visualizer.show_conf(final_sol[0])
        # np.save('sol' ,np.array(final_sol))
    except:
        print('No solution found')