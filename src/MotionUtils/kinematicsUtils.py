import numpy as np
from math import sin, cos, atan2, acos, pi, sqrt, asin, atan
from scipy.spatial.transform import Rotation as R


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

T_AB_UR3E_to_UR5E = np.eye(4)

T_AB_UR3E_to_UR5E[0, 3] = -0.12  # Distance along the x-axis from UR3E to UR5E
T_AB_UR3E_to_UR5E[1, 3] = -0.07  # Distance along the y-axis from UR3E to UR5E
T_AB_UR3E_to_UR5E[2, 3] = -0.00   # Distance along the z-axis from UR3E to UR5E

CAMERA_EE_DISPLACEMENT = [0,-0.13,0]
PLATE_EE_DISPLACEMENT = [0,0,0.12]
ACCEPTABLE_DISPLACEMENT = 0.02 # was 0.05 originally

def toView(conf, end = "\n"):
    return [round(a, 2) for a in conf]

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
    xyz_coords = np.array([transform[0,3],transform[1,3],transform[2,3]]) # xyz coordinates format
    return np.array([round(num, 4) for num in xyz_coords.tolist()])

def forward_kinematic(DH_matrix, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    """
    Compute the forward kinematic solution for the given joint angles.

    Parameters:
    DH_matrix (numpy.matrix): The DH parameters matrix for the robot.
    edges (numpy.matrix): The joint angles or displacements.

    Returns:
    tuple: The position (x, y, z) and orientation (rx, ry, rz) of the end-effector.
    """
    t01 = mat_transform_DH(DH_matrix, 1, edges)
    t12 = mat_transform_DH(DH_matrix, 2, edges)
    t23 = mat_transform_DH(DH_matrix, 3, edges)
    t34 = mat_transform_DH(DH_matrix, 4, edges)
    t45 = mat_transform_DH(DH_matrix, 5, edges)
    t56 = mat_transform_DH(DH_matrix, 6, edges)

    transform = t01 * t12 * t23 * t34 * t45 * t56

    x, y, z = transform[0, 3], transform[1, 3], transform[2, 3]

    rotation_matrix = transform[:3, :3]

    r = R.from_matrix(rotation_matrix)
    rx, ry, rz = r.as_rotvec()

    return (round(x, 4), round(y, 4), round(z, 4), round(rx, 4), round(ry, 4), round(rz, 4))

def forward_kinematic_matrix(DH_matrix, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    t01 = mat_transform_DH(DH_matrix, 1, edges)
    t12 = mat_transform_DH(DH_matrix, 2, edges)
    t23 = mat_transform_DH(DH_matrix, 3, edges)
    t34 = mat_transform_DH(DH_matrix, 4, edges)
    t45 = mat_transform_DH(DH_matrix, 5, edges)
    t56 = mat_transform_DH(DH_matrix, 6, edges)
    transform = t01 * t12 * t23 * t34 * t45 * t56
    return transform

def balanced_config_autocomplete(config, joint_4_direction = -1, bias = (0,2.5)):
    c = config
    d = 1 if joint_4_direction > 0 else -1
    return [config[0], config[1], config[2], -(c[1]+c[2]) + d*np.deg2rad(bias[1]) - np.pi,d*np.pi/2, np.pi/2 + np.deg2rad(bias[0])]

def flat_inverse_kinematic_solution(DH_matrix, goal_xyz,):
    initial_config = balanced_config_autocomplete((0, -np.pi/2, 0, 0))

def inverse_kinematic_solution(DH_matrix, transform_matrix,):

    theta = np.matrix(np.zeros((6, 8)))
    # theta 1
    T06 = transform_matrix

    P05 = T06 * np.matrix([[0], [0], [-DH_matrix[5, 2]], [1]])
    psi = atan2(P05[1], P05[0])
    # phi = acos((DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2]) / sqrt(P05[0] ** 2 + P05[1] ** 2)) # possible error here - changed to line below
    phi = acos(np.clip((DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2]) / sqrt(P05[0] ** 2 + P05[1] ** 2), -1, 1))
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
        T60 = np.linalg.inv(T06)
        th = atan2((-T60[1, 0] * sin(theta[0, i]) + T60[1, 1] * cos(theta[0, i])),
                   (T60[0, 0] * sin(theta[0, i]) - T60[0, 1] * cos(theta[0, i])))
        theta[5, i:i + 2] = th

    # theta 3
    for i in {0, 2, 4, 6}:
        T01 = mat_transform_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transform_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transform_DH(DH_matrix, 6, theta[:, i])
        T14 = np.linalg.inv(T01) * T06 * np.linalg.inv(T45 * T56)
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
        T14 = np.linalg.inv(T01) * T06 * np.linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])

        theta[1, i] = atan2(-P13[1], -P13[0]) - asin(
            -DH_matrix[2, 0] * sin(theta[2, i]) / sqrt(P13[0] ** 2 + P13[1] ** 2)
        )
        T32 = np.linalg.inv(mat_transform_DH(DH_matrix, 3, theta[:, i]))
        T21 = np.linalg.inv(mat_transform_DH(DH_matrix, 2, theta[:, i]))
        T34 = T32 * T21 * T14
        theta[3, i] = atan2(T34[1, 0], T34[0, 0])
    return theta

def inverse_kinematics_solutions_endpos(tx, ty, tz, alpha = -np.pi, beta = 0.0, gamma = 0):
    transform = np.matrix([[cos(beta) * cos(gamma), sin(alpha) * sin(beta)*cos(gamma) - cos(alpha)*sin(gamma),
                    cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma), tx],
                    [cos(beta)* sin(gamma), sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma),
                     cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma), ty],
                    [-sin(beta), sin(alpha)*cos(beta), cos(alpha)*cos(beta), tz],
                     [0, 0,0,1]])

    IKS = inverse_kinematic_solution(DH_matrix_ur5e, transform)
    candidate_sols = []
    for i in range(IKS.shape[1]):
        candidate_sols.append(IKS[:, i])
    return np.array(candidate_sols)

def get_valid_inverse_solutions(DH_matrix, tx, ty, tz, alpha = -np.pi, beta = 0.0, gamma = 0):
    candidate_sols = inverse_kinematics_solutions_endpos(tx, ty, tz, alpha, beta , gamma)
    sols = []
    for candidate_sol in candidate_sols:
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
        transform = forward_kinematic_matrix(DH_matrix, sol)
        diff = np.linalg.norm(np.array([transform[0,3],transform[1,3],transform[2,3]])-
                              np.array([tx,ty,tz]))
        if diff < ACCEPTABLE_DISPLACEMENT:
            final_sol.append(sol)
    final_sol = np.array(final_sol)
    return [[c[0] for c in p] for p in final_sol]

def assure_homogeneous(coord):
    if(len(coord) == 3):
        return np.append(coord,1)
    return coord

def camera_from_ee(coord):
    array =  np.array([coord[0] + CAMERA_EE_DISPLACEMENT[0], coord[1] + CAMERA_EE_DISPLACEMENT[1], coord[2] + CAMERA_EE_DISPLACEMENT[2], 1])
    return np.array([round(num, 4) for num in array.tolist()])

def plate_from_ee(coord):
    array = np.array([coord[0] + PLATE_EE_DISPLACEMENT[0], coord[1] + PLATE_EE_DISPLACEMENT[1], coord[2] + PLATE_EE_DISPLACEMENT[2], 1])
    return np.array([round(num, 4) for num in array.tolist()])

def ur3e_effector_to_home(ur3e_joints, local_coords = [0,0,0,1]):
    local_coords = assure_homogeneous(local_coords)
    mat_end_to_ur3_base = forward_kinematic_matrix(DH_matrix_ur3e, ur3e_joints)
    inter = np.ravel(np.dot(mat_end_to_ur3_base, np.asarray(local_coords)))
    position = np.ravel(np.dot(T_AB_UR3E_to_UR5E, inter))
    return np.array([round(num, 4) for num in position.tolist()])

def ur5e_effector_to_home(ur5e_joints, local_coords = [0,0,0,1]):
    local_coords = assure_homogeneous(local_coords)
    mat_end_to_ur5_base = forward_kinematic_matrix(DH_matrix_ur5e, ur5e_joints)
    array = np.ravel(np.dot(mat_end_to_ur5_base, local_coords))
    return np.array([round(num, 4) for num in array.tolist()])

def calculate_assistant_robot_path(task_path):
    alpha, beta, gamma = -np.pi, 0.0, 0.0
    safety_distance = 0.12  # add 0.25 [meters] safety distance above the plate for safety concerns

    assistant_path = []

    for index, conf in enumerate(task_path):
        ur3e_position_in_home = ur3e_effector_to_home(conf) # possible BUG
        print([round(q,2) for q in ur3e_position_in_home])

        try:
            tx, ty, tz = ur3e_position_in_home[0], ur3e_position_in_home[1], ur3e_position_in_home[2] + safety_distance
            if index == len(task_path) - 1:
                x = 1
            ur5e_joint_angles = get_valid_inverse_solutions(DH_matrix_ur5e, tx, ty, tz, alpha, beta, gamma)[0]
            print(toView(ur5e_joint_angles))
            if ur5e_joint_angles is not None:
                assistant_path.append(ur5e_joint_angles)
                xyz_position_ur5e_end_effector = forward_kinematic_solution(DH_matrix_ur5e, ur5e_joint_angles)
                print(f"UR3E position: {toView(ur3e_position_in_home[:3])}, config: {conf}\n",
                        f"UR5E position: {toView(xyz_position_ur5e_end_effector)}, config: {toView(ur5e_joint_angles)}\n")
            else:
                print(f"{index}'th config has no corresponding solution.")
                assistant_path.append(None)
        except Exception as e:
            print(f"Error in computing inverse kinematics solution: {e}")
    return assistant_path
