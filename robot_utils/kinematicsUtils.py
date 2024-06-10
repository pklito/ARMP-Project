import numpy as np
from math import sin, cos, atan2

# Define the tool length and DH matrices for different UR arms
tool_length = 0.135  # [m]

DH_matrix_ur3e = np.matrix([[0, np.pi / 2.0, 0.15185],
                       [-0.24355, 0, 0],
                       [-0.2132, 0, 0],
                       [0, np.pi / 2.0, 0.13105],
                       [0, -np.pi / 2.0, 0.08535],
                       [0, 0, 0.0921 + tool_length]])

DH_matrix_ur5e = np.matrix([[0, np.pi / 2.0, 0.1625],
                       [-0.425, 0, 0],
                       [-0.3922, 0, 0],
                       [0, np.pi / 2.0, 0.1333],
                       [0, -np.pi / 2.0, 0.0997],
                       [0, 0, 0.0996]])

T_UR3E_to_UR5E_Camera = np.eye(4)
T_UR5E_Camera_to_UR3E = np.eye(4)
T_AB_UR3E_to_UR5E = np.eye(4)

T_UR3E_to_UR5E_Camera[0, 3] = 1.35  # Distance along the x-axis from UR3E to UR5E Camera
T_UR3E_to_UR5E_Camera[1, 3] = 0.07  # Distance along the y-axis from UR3E to UR5E Camera
T_UR3E_to_UR5E_Camera[2, 3] = 0.1   # Distance along the z-axis from UR3E to UR5E Camera

T_AB_UR3E_to_UR5E[0, 3] = -1.35  # Distance along the x-axis from UR3E to UR5E
T_AB_UR3E_to_UR5E[1, 3] = -0.07  # Distance along the y-axis from UR3E to UR5E
T_AB_UR3E_to_UR5E[2, 3] = -0.00   # Distance along the z-axis from UR3E to UR5E

T_UR5E_Camera_to_UR3E = np.linalg.inv(T_UR3E_to_UR5E_Camera)
T_BA_UR5E_to_UR3E = np.linalg.inv(T_AB_UR3E_to_UR5E)

def mat_transform_DH(DH_matrix, n, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    """
    Compute the transformation matrix for the nth joint using Denavit-Hartenberg parameters.

    Parameters:
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
    transform = t01 * t12 * t23 * t34 * t45 * t56
    return np.array([transform[0,3],transform[1,3],transform[2,3]]) # xyz coordinates format

def forward_kinematic_matrix(DH_matrix, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    t01 = mat_transform_DH(DH_matrix, 1, edges)
    t12 = mat_transform_DH(DH_matrix, 2, edges)
    t23 = mat_transform_DH(DH_matrix, 3, edges)
    t34 = mat_transform_DH(DH_matrix, 4, edges)
    t45 = mat_transform_DH(DH_matrix, 5, edges)
    t56 = mat_transform_DH(DH_matrix, 6, edges)
    transform = t01 * t12 * t23 * t34 * t45 * t56
    return transform

def camera_from_ee(coord):
    return [coord[0], coord[1] - 0.1, coord[2], 1]
def plate_from_ee(coord):
    return [coord[0], coord[1], coord[2] + 0.1, 1]

def assure_homogeneous(coord):
    if(len(coord) == 3):
        return np.append(coord,1)
    return coord

def ur3e_effector_to_home(ur3e_joints, local_coords = [0,0,0,1]):
    local_coords = assure_homogeneous(local_coords)
    mat_end_to_ur3_base = forward_kinematic_matrix(DH_matrix_ur3e, ur3e_joints)
    inter = np.ravel(np.dot(mat_end_to_ur3_base, np.asarray(local_coords)))
    position = np.ravel(np.dot(T_AB_UR3E_to_UR5E, inter))
    return position

def ur5e_effector_to_home(ur5e_joints, local_coords = [0,0,0,1]):
    local_coords = assure_homogeneous(local_coords)
    mat_end_to_ur5_base = forward_kinematic_matrix(DH_matrix_ur5e, ur5e_joints)
    return np.ravel(np.dot(mat_end_to_ur5_base,local_coords))
