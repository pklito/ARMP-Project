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


def calculate_plate_center(ur3e_joint_angles, DH_matrix=DH_matrix_ur3e):
    """
    Calculate the center of a plate using the UR3e arm's end-effector position in UR3E's frame.
    """
    xyz_position_ur3e_end_effector = forward_kinematic_solution(DH_matrix, ur3e_joint_angles)
    dx, dy, dz = 0.1, 0.0, 0.0 # TODO: perform the calculations between the ee and the plate center, adjust as necessary! dz or dx should be 0.1
    plate_center = xyz_position_ur3e_end_effector[0] + dx, xyz_position_ur3e_end_effector[1] + dy, xyz_position_ur3e_end_effector[2] + dz
    return plate_center

def calculate_ur3e_coords_in_camera_frame(coords):
    return transition_coordinates(coords, T_UR3E_to_UR5E_Camera)

# def calculate_ur3e_coords_in_camera_frame(coords):
#     return transition_coordinates(coords, tab_u)

def pixel_coords_to_world_coords(xyz_coords, intrinsic_matrix):
    xyz_homogeneous = np.array(xyz_coords)
    pixel_coords_homogeneous = np.dot(intrinsic_matrix, xyz_homogeneous)
    pixel_coords = pixel_coords_homogeneous / pixel_coords_homogeneous[2]
    return pixel_coords[:3]

def calculate_intrinsic_matrix(camera_params):
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


def get_ball_position_from_camera(ball_pixel_x = 320, ball_pixel_y = 240):
    camera = CameraStreamer()
    if not camera.depth_frame:
        return None

    depth = camera.depth_frame.get_distance(ball_pixel_x, ball_pixel_y)
    depth_intrinsics = camera.depth_frame.profile.as_video_stream_profile().intrinsics

    # Convert pixel coordinates to world coordinates (from the Camera's Perspective!)
    ball_position = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [ball_pixel_x, ball_pixel_y], depth)
    return np.array(ball_position)
