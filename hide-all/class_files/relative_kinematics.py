import numpy as np

def transformation_matrix(rotation_angles, translation_vector):
    alpha, beta, gamma = rotation_angles
    tx, ty, tz = translation_vector
    
    Rz = np.array([
        [np.cos(alpha), -np.sin(alpha), 0, 0],
        [np.sin(alpha),  np.cos(alpha), 0, 0],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])
    
    Ry = np.array([
        [np.cos(beta), 0, np.sin(beta), 0],
        [0,            1, 0,            0],
        [-np.sin(beta), 0, np.cos(beta), 0],
        [0,            0, 0,            1]
    ])
    
    Rx = np.array([
        [1, 0,             0,            0],
        [0, np.cos(gamma), -np.sin(gamma), 0],
        [0, np.sin(gamma),  np.cos(gamma), 0],
        [0, 0,             0,            1]
    ])
    
    R = Rz @ Ry @ Rx # @ is used for matrix multiplication instead of np.dot()
    
    T = np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])
    
    return T @ R

# Example usage for two robots
rotation_angles_A = (np.pi / 4, np.pi / 4, np.pi / 4)  # Example angles for Robot A
translation_vector_A = (1, 1, 1)                      # Example translation for Robot A

rotation_angles_B = (np.pi / 2, np.pi / 2, np.pi / 2)  # Example angles for Robot B
translation_vector_B = (2, 2, 2)                      # Example translation for Robot B

T_A = transformation_matrix(rotation_angles_A, translation_vector_A)
T_B = transformation_matrix(rotation_angles_B, translation_vector_B)

T_B_inv = np.linalg.inv(T_B)
T_A_relative_to_B = T_B_inv @ T_A

# Transform a point from Robot A's frame to Robot B's frame
point_A = np.array([1, 0, 0, 1])  # Example point in Robot A's frame (homogeneous coordinates)
point_B = T_A_relative_to_B @ point_A

# Extract the transformed point
transformed_point_B = point_B[:3]  # Extract the x, y, z coordinates
print("Transformed point in Robot B's frame:", transformed_point_B)
