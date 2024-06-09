import numpy as np
from math import sin, cos, atan2
from realsense_camera.CameraStreamer import *

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

T_AB_UR3E_to_UR5E[0, 3] = 1.35  # Distance along the x-axis from UR3E to UR5E
T_AB_UR3E_to_UR5E[1, 3] = 0.07  # Distance along the y-axis from UR3E to UR5E
T_AB_UR3E_to_UR5E[2, 3] = 0.00   # Distance along the z-axis from UR3E to UR5E

T_UR5E_Camera_to_UR3E = np.linalg.inv(T_UR3E_to_UR5E_Camera)
T_BA_UR5E_to_UR3E = np.linalg.inv(T_AB_UR3E_to_UR5E)

def transition_coordinates(point, transformation_matrix):
    """
    Transition coordinates from one frame to another using a transformation matrix.
    """
    transformed_point = np.dot(transformation_matrix, np.append(point, 1))
    return transformed_point[:3] # Return only the XYZ coordinates


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


# Note: the following code could have multiple errors due to a minus / plus sign! try changing the signs of trasformations from one frame to another to receive a solution - TODO: Fix this!

def calculate_plate_center(ur3e_joint_angles, DH_matrix=DH_matrix_ur3e):
    """
    Calculate the center of a plate using the UR3e arm's end-effector position in UR3E's frame.
    """
    xyz_position_ur3e_end_effector = forward_kinematic_solution(DH_matrix, ur3e_joint_angles)
    dx, dy, dz = 0.1, 0.0, 0.0 # TODO - Fix distances as necessary, there could be a mess up in which axis should be added to
    plate_center = xyz_position_ur3e_end_effector[0] + dx, xyz_position_ur3e_end_effector[1] + dy, xyz_position_ur3e_end_effector[2] + dz
    return plate_center # in UR3E World coordinates

def transform_UR3E_to_UR5E(position_UR3E):
    # Transformation matrix from UR3E to UR5E - TODO: Fix distances if wrong
    T_UR3E_to_UR5E = np.eye(4)
    T_UR3E_to_UR5E[0, 3] = 1.35  # Distance along the x-axis from UR3E Base to UR5E Base
    T_UR3E_to_UR5E[1, 3] = 0.07  # Distance along the y-axis from UR3E Base to UR5E Base
    T_UR3E_to_UR5E[2, 3] = 0.00   # Distance along the z-axis from UR3E Base to UR5E Base
    position_in_ur5e_ = np.dot(T_UR3E_to_UR5E, np.append(position_UR3E, 1))
    return position_in_ur5e_[:3]

def transform_UR5E_End_Effector_to_Camera(position_UR5E):
    # Transformation matrix from UR5E to Camera - TODO: Fix distances if wrong
    T_UR5E_to_Camera = np.eye(4)
    T_UR5E_to_Camera[0, 3] = 0.00  # Distance along the x-axis from UR5E End-Effector to UR5E mounted Camera
    T_UR5E_to_Camera[1, 3] = 0.00  # Distance along the y-axis from UR5E End-Effector to UR5E mounted Camera
    T_UR5E_to_Camera[2, 3] = 0.10  # Distance along the z-axis from UR5E End-Effector to UR5E mounted Camera
    position_Camera_ = np.dot(T_UR5E_to_Camera, np.append(position_UR5E, 1))
    return position_Camera_[:3]

def transform_UR3E_to_Camera(position_UR3E, ur5e_joints):
    ur5e_end_effector_position = forward_kinematic_solution(DH_matrix_ur5e,ur5e_joints) # this is in relation to the UR5E Base
    position_UR5E = transform_UR3E_to_UR5E(position_UR3E) # this is the position of a point from the UR3E worls in the UR5E world
    position_in_end_effector_world = position_UR5E - ur5e_end_effector_position # basically considering the end effector of UR5E as the Origin
    return transform_UR5E_End_Effector_to_Camera(position_in_end_effector_world)


# def pixel_coords_to_world_coords(xyz_coords, intrinsic_matrix):
#     xyz_homogeneous = np.array(xyz_coords)
#     pixel_coords_homogeneous = np.dot(intrinsic_matrix, xyz_homogeneous)
#     pixel_coords = pixel_coords_homogeneous / pixel_coords_homogeneous[2]
#     return pixel_coords[:3]

# def calculate_intrinsic_matrix(camera_params):
    fov_horizontal_rad = camera_params['fov_horizontal_rad']
    fov_vertical_rad = camera_params['fov_vertical_rad']
    image_width = camera_params['image_width']
    image_height = camera_params['image_height']

    focal_length_x = (image_width / 2) / np.tan(fov_horizontal_rad / 2)
    focal_length_y = (image_height / 2) / np.tan(fov_vertical_rad / 2)

    c_x = image_width / 2
    c_y = image_height / 2

    intrinsic_matrix = np.array([[focal_length_x, 0, c_x],
                                 [0, focal_length_y, c_y],
                                 [0, 0, 1]])

    return intrinsic_matrix

