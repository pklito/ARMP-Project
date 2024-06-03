from flask import config
import numpy as np
from ur_ikfast import ur_kinematics
from ur_ikfast.ur_kinematics import best_ik_sol, pose_quaternion_from_matrix
from UR_Params import UR5e_PARAMS, UR3e_PARAMS, Transform
from building_blocks import Building_Blocks

# Define the tool length and DH matrices for different UR arms
tool_length = 0.135  # [m]

DH_matrices = {
    "ur3e": np.matrix([[0, np.pi / 2.0, 0.15185],
                       [-0.24355, 0, 0],
                       [-0.2132, 0, 0],
                       [0, np.pi / 2.0, 0.13105],
                       [0, -np.pi / 2.0, 0.08535],
                       [0, 0, 0.0921 + tool_length]]),
    "ur5e": np.matrix([[0, np.pi / 2.0, 0.1625],
                       [-0.425, 0, 0],
                       [-0.3922, 0, 0],
                       [0, np.pi / 2.0, 0.1333],
                       [0, -np.pi / 2.0, 0.0997],
                       [0, 0, 0.0996 + tool_length]])
}

def valid_inverse_solutions(bb, ur_arm, tx, ty, tz, w, alpha, beta, gamma, q_guess=np.zeros(6), weights=np.ones(6)):
    """
    Get valid inverse kinematics solutions that are collision-free.

    Computes the inverse kinematics solutions for a given target end-effector
    position and orientation, ensuring that the resulting joint configurations
    are collision-free.

    :param tx: Target x position of the end-effector.
    :param ty: Target y position of the end-effector.
    :param tz: Target z position of the end-effector.
    :param w: Weight parameter for the end-effector position. Set to 1.0 if not specified otherwise.
    :param alpha: Target roll angle (orientation) of the end-effector.
    :param beta: Target pitch angle (orientation) of the end-effector.
    :param gamma: Target yaw angle (orientation) of the end-effector.
    :param bb: Building_Blocks object for collision checking.
    :param ur_arm: Specific UR arm object providing inverse kinematics capabilities.
    :param q_guess: Initial guess for the joint angles.
    :param weights: Weights for selecting the best IK solution.
    :return: Best valid joint angle solution that achieves the specified end-effector
             pose and is collision-free.
    """
    # Assume w is set to 1.0 if not specified otherwise - TODO: research how to retrieve this
    # w = 1.0
    target_pose = np.array([tx, ty, tz, w, alpha, beta, gamma])

    q_solutions = ur_arm.inverse(target_pose, True)
    if q_solutions is None:
        return None

    valid_solutions = []
    for q in q_solutions:
        if not bb.is_in_collision(q):
            valid_solutions.append(q)

    return best_ik_sol(valid_solutions, q_guess, weights)


def forward_kinematic_solution(joint_angles, ur_arm):
    """
    Compute the forward kinematics.

    :param joint_angles: List of joint angles.
    :param ur_arm: Specific UR arm object.
    :return: End effector pose.
    """
    return ur_arm.forward(joint_angles)


def transition_coordinates(point_A):
    """
    transition from UR3E coordinates to UR5E coordinates
    """
    dx = -0.13  # Distance along the x-axis from frame A to frame B - TODO: measure
    dy = -0.37  # Distance along the y-axis from frame A to frame B - TODO: measure
    dz = 0.0

    # Create the transformation matrix
    T_AB = np.eye(4)
    T_AB[0, 3] = dx
    T_AB[1, 3] = dy
    T_AB[2, 3] = dz

    point_B = np.dot(T_AB, point_A)
    return point_B[:3]


def calculate_camera_robot_transitions(ur3e_arm, ur5e_arm, task_robot_path_configs):

    ur_params_ur3e = UR3e_PARAMS(inflation_factor=1)
    ur_params_ur5e = UR5e_PARAMS(inflation_factor=1)

    transform_ur5e = Transform(ur_params_ur5e)
    transform_ur3e = Transform(ur_params_ur3e)

    bb_ur5e = Building_Blocks(transform=transform_ur5e, ur_params=ur_params_ur5e, env=None, resolution=0.1, p_bias=0.05)
    bb_ur3e = Building_Blocks(transform=transform_ur3e, ur_params=ur_params_ur3e, env=None, resolution=0.1, p_bias=0.05) # should implement seperate collision functions

    # calculate FK to get the position of the end effector on the UR3E
    for index, conf in enumerate(task_robot_path_configs):
        quaternion_ur3e = ur3e_arm.forward(conf)
        xyz_position_ur3e = quaternion_ur3e[:3]

        # Transition to UR5E frame
        position_in_ur5e_frame = transition_coordinates(np.append(xyz_position_ur3e, 1))[:3]
        ur5e_pose = np.append(position_in_ur5e_frame, (quaternion_ur3e[3], quaternion_ur3e[4], quaternion_ur3e[5], quaternion_ur3e[6])) # TODO: consider using custom values for orientation
        safety_distance = 0.25  # add 0.25 [meters] safety distance above the plate for safety concerns
        ur5e_pose[2] += safety_distance

        try:
            # Use inverse kinematics function for UR5E to find joint angles
            ur5e_joint_angles = valid_inverse_solutions(bb_ur5e, ur5e_arm, *ur5e_pose)

            if ur5e_joint_angles is not None:
                quaternion_ur5e = ur5e_arm.forward(ur5e_joint_angles)
                xyz_position_ur5e_end_effector = quaternion_ur5e[:3]
                print(f"UR3E position: {xyz_position_ur3e}, config: {conf}\n",
                        f"UR5E position: {xyz_position_ur5e_end_effector}, config: {ur5e_joint_angles}\n")
            else:
                print(f"{index}'th config has no corresponding solution.")
        except Exception as e:
            print(f"Error in finding inverse kinematics solution: {e}")

ur3e_arm = ur_kinematics.URKinematics('ur3e')
ur5e_arm = ur_kinematics.URKinematics('ur5e')
task_robot_path_configs = [np.array([-0.17453293, -1.3962634 ,  0.17453293, -1.22173048,  0.17453293,
    0.        ]), np.array([-0.17104227, -1.39277274,  0.17366026, -1.22173048,  0.17104227,
    0.        ]), np.array([-0.16755161, -1.38928208,  0.1727876 , -1.22173048,  0.16755161,
    0.        ]), np.array([-0.16406095, -1.38579143,  0.17191493, -1.22173048,  0.16406095,
    0.        ]), np.array([-0.16057029, -1.38230077,  0.17104227, -1.22173048,  0.16057029,
    0.        ]), np.array([-0.15707963, -1.37881011,  0.1701696 , -1.22173048,  0.15707963,
    0.        ]), np.array([-0.15358897, -1.37531945,  0.16929694, -1.22173048,  0.15358897,
    0.        ]), np.array([-0.15009832, -1.37182879,  0.16842427, -1.22173048,  0.15009832,
    0.        ]), np.array([-0.14660766, -1.36833813,  0.16755161, -1.22173048,  0.14660766,
    0.        ]), np.array([-0.143117  , -1.36484748,  0.16667894, -1.22173048,  0.143117  ,
    0.        ]), np.array([-0.13962634, -1.36135682,  0.16580628, -1.22173048,  0.13962634,
    0.        ]), np.array([-0.13613568, -1.35786616,  0.16493361, -1.22173048,  0.13613568,
    0.        ]), np.array([-0.13264502, -1.3543755 ,  0.16406095, -1.22173048,  0.13264502,
    0.        ]), np.array([-0.12915436, -1.35088484,  0.16318829, -1.22173048,  0.12915436,
    0.        ]), np.array([-0.12566371, -1.34739418,  0.16231562, -1.22173048,  0.12566371,
    0.        ]), np.array([-0.12217305, -1.34390352,  0.16144296, -1.22173048,  0.12217305,
    0.        ]), np.array([-0.11868239, -1.34041287,  0.16057029, -1.22173048,  0.11868239,
    0.        ]), np.array([-0.11519173, -1.33692221,  0.15969763, -1.22173048,  0.11519173,
    0.        ]), np.array([-0.11170107, -1.33343155,  0.15882496, -1.22173048,  0.11170107,
    0.        ]), np.array([-0.10821041, -1.32994089,  0.1579523 , -1.22173048,  0.10821041,
    0.        ]), np.array([-0.10471976, -1.32645023,  0.15707963, -1.22173048,  0.10471976,
    0.        ]), np.array([-0.1012291 , -1.32295957,  0.15620697, -1.22173048,  0.1012291 ,
    0.        ]), np.array([-0.09773844, -1.31946891,  0.1553343 , -1.22173048,  0.09773844,
    0.        ]), np.array([-0.09424778, -1.31597826,  0.15446164, -1.22173048,  0.09424778,
    0.        ]), np.array([-0.09075712, -1.3124876 ,  0.15358897, -1.22173048,  0.09075712,
    0.        ]), np.array([-0.08726646, -1.30899694,  0.15271631, -1.22173048,  0.08726646,
    0.        ]), np.array([-0.0837758 , -1.30550628,  0.15184364, -1.22173048,  0.0837758 ,
    0.        ]), np.array([-0.08028515, -1.30201562,  0.15097098, -1.22173048,  0.08028515,
    0.        ]), np.array([-0.07679449, -1.29852496,  0.15009832, -1.22173048,  0.07679449,
    0.        ]), np.array([-0.07330383, -1.2950343 ,  0.14922565, -1.22173048,  0.07330383,
    0.        ]), np.array([-0.06981317, -1.29154365,  0.14835299, -1.22173048,  0.06981317,
    0.        ]), np.array([-0.06632251, -1.28805299,  0.14748032, -1.22173048,  0.06632251,
    0.        ]), np.array([-0.06283185, -1.28456233,  0.14660766, -1.22173048,  0.06283185,
    0.        ]), np.array([-0.05934119, -1.28107167,  0.14573499, -1.22173048,  0.05934119,
    0.        ]), np.array([-0.05585054, -1.27758101,  0.14486233, -1.22173048,  0.05585054,
    0.        ]), np.array([-0.05235988, -1.27409035,  0.14398966, -1.22173048,  0.05235988,
    0.        ]), np.array([-0.04886922, -1.2705997 ,  0.143117  , -1.22173048,  0.04886922,
    0.        ]), np.array([-0.04537856, -1.26710904,  0.14224433, -1.22173048,  0.04537856,
    0.        ]), np.array([-0.0418879 , -1.26361838,  0.14137167, -1.22173048,  0.0418879 ,
    0.        ]), np.array([-0.03839724, -1.26012772,  0.140499  , -1.22173048,  0.03839724,
    0.        ]), np.array([-0.03490659, -1.25663706,  0.13962634, -1.22173048,  0.03490659,
    0.        ]), np.array([-0.03141593, -1.2531464 ,  0.13875368, -1.22173048,  0.03141593,
    0.        ]), np.array([-0.02792527, -1.24965574,  0.13788101, -1.22173048,  0.02792527,
    0.        ]), np.array([-0.02443461, -1.24616509,  0.13700835, -1.22173048,  0.02443461,
    0.        ]), np.array([-0.02094395, -1.24267443,  0.13613568, -1.22173048,  0.02094395,
    0.        ]), np.array([-0.01745329, -1.23918377,  0.13526302, -1.22173048,  0.01745329,
    0.        ]), np.array([-0.01396263, -1.23569311,  0.13439035, -1.22173048,  0.01396263,
    0.        ]), np.array([-0.01047198, -1.23220245,  0.13351769, -1.22173048,  0.01047198,
    0.        ]), np.array([-0.00698132, -1.22871179,  0.13264502, -1.22173048,  0.00698132,
    0.        ]), np.array([-0.00349066, -1.22522113,  0.13177236, -1.22173048,  0.00349066,
    0.        ]), np.array([ 0.        , -1.22173048,  0.13089969, -1.22173048,  0.        ,
    0.        ]), np.array([ 0.00349066, -1.21823982,  0.13002703, -1.22173048, -0.00349066,
    0.        ]), np.array([ 0.00698132, -1.21474916,  0.12915436, -1.22173048, -0.00698132,
    0.        ]), np.array([ 0.01047198, -1.2112585 ,  0.1282817 , -1.22173048, -0.01047198,
    0.        ]), np.array([ 0.01396263, -1.20776784,  0.12740904, -1.22173048, -0.01396263,
    0.        ]), np.array([ 0.01745329, -1.20427718,  0.12653637, -1.22173048, -0.01745329,
    0.        ]), np.array([ 0.02094395, -1.20078653,  0.12566371, -1.22173048, -0.02094395,
    0.        ]), np.array([ 0.02443461, -1.19729587,  0.12479104, -1.22173048, -0.02443461,
    0.        ]), np.array([ 0.02792527, -1.19380521,  0.12391838, -1.22173048, -0.02792527,
    0.        ]), np.array([ 0.03141593, -1.19031455,  0.12304571, -1.22173048, -0.03141593,
    0.        ]), np.array([ 0.03490659, -1.18682389,  0.12217305, -1.22173048, -0.03490659,
    0.        ]), np.array([ 0.03839724, -1.18333323,  0.12130038, -1.22173048, -0.03839724,
    0.        ]), np.array([ 0.0418879 , -1.17984257,  0.12042772, -1.22173048, -0.0418879 ,
    0.        ]), np.array([ 0.04537856, -1.17635192,  0.11955505, -1.22173048, -0.04537856,
    0.        ]), np.array([ 0.04886922, -1.17286126,  0.11868239, -1.22173048, -0.04886922,
    0.        ]), np.array([ 0.05235988, -1.1693706 ,  0.11780972, -1.22173048, -0.05235988,
    0.        ]), np.array([ 0.05585054, -1.16587994,  0.11693706, -1.22173048, -0.05585054,
    0.        ]), np.array([ 0.05934119, -1.16238928,  0.1160644 , -1.22173048, -0.05934119,
    0.        ]), np.array([ 0.06283185, -1.15889862,  0.11519173, -1.22173048, -0.06283185,
    0.        ]), np.array([ 0.06632251, -1.15540796,  0.11431907, -1.22173048, -0.06632251,
    0.        ]), np.array([ 0.06981317, -1.15191731,  0.1134464 , -1.22173048, -0.06981317,
    0.        ]), np.array([ 0.07330383, -1.14842665,  0.11257374, -1.22173048, -0.07330383,
    0.        ]), np.array([ 0.07679449, -1.14493599,  0.11170107, -1.22173048, -0.07679449,
    0.        ]), np.array([ 0.08028515, -1.14144533,  0.11082841, -1.22173048, -0.08028515,
    0.        ]), np.array([ 0.0837758 , -1.13795467,  0.10995574, -1.22173048, -0.0837758 ,
    0.        ]), np.array([ 0.08726646, -1.13446401,  0.10908308, -1.22173048, -0.08726646,
    0.        ]), np.array([ 0.09075712, -1.13097336,  0.10821041, -1.22173048, -0.09075712,
    0.        ]), np.array([ 0.09424778, -1.1274827 ,  0.10733775, -1.22173048, -0.09424778,
    0.        ]), np.array([ 0.09773844, -1.12399204,  0.10646508, -1.22173048, -0.09773844,
    0.        ]), np.array([ 0.1012291 , -1.12050138,  0.10559242, -1.22173048, -0.1012291 ,
    0.        ]), np.array([ 0.10471976, -1.11701072,  0.10471976, -1.22173048, -0.10471976,
    0.        ]), np.array([ 0.10821041, -1.11352006,  0.10384709, -1.22173048, -0.10821041,
    0.        ]), np.array([ 0.11170107, -1.1100294 ,  0.10297443, -1.22173048, -0.11170107,
    0.        ]), np.array([ 0.11519173, -1.10653875,  0.10210176, -1.22173048, -0.11519173,
    0.        ]), np.array([ 0.11868239, -1.10304809,  0.1012291 , -1.22173048, -0.11868239,
    0.        ]), np.array([ 0.12217305, -1.09955743,  0.10035643, -1.22173048, -0.12217305,
    0.        ]), np.array([ 0.12566371, -1.09606677,  0.09948377, -1.22173048, -0.12566371,
    0.        ]), np.array([ 0.12915436, -1.09257611,  0.0986111 , -1.22173048, -0.12915436,
    0.        ]), np.array([ 0.13264502, -1.08908545,  0.09773844, -1.22173048, -0.13264502,
    0.        ]), np.array([ 0.13613568, -1.08559479,  0.09686577, -1.22173048, -0.13613568,
    0.        ]), np.array([ 0.13962634, -1.08210414,  0.09599311, -1.22173048, -0.13962634,
    0.        ]), np.array([ 0.143117  , -1.07861348,  0.09512044, -1.22173048, -0.143117  ,
    0.        ]), np.array([ 0.14660766, -1.07512282,  0.09424778, -1.22173048, -0.14660766,
    0.        ]), np.array([ 0.15009832, -1.07163216,  0.09337511, -1.22173048, -0.15009832,
    0.        ]), np.array([ 0.15358897, -1.0681415 ,  0.09250245, -1.22173048, -0.15358897,
    0.        ]), np.array([ 0.15707963, -1.06465084,  0.09162979, -1.22173048, -0.15707963,
    0.        ]), np.array([ 0.16057029, -1.06116019,  0.09075712, -1.22173048, -0.16057029,
    0.        ]), np.array([ 0.16406095, -1.05766953,  0.08988446, -1.22173048, -0.16406095,
    0.        ]), np.array([ 0.16755161, -1.05417887,  0.08901179, -1.22173048, -0.16755161,
    0.        ]), np.array([ 0.17104227, -1.05068821,  0.08813913, -1.22173048, -0.17104227,
    0.        ]), np.array([ 0.17453293, -1.04719755,  0.08726646, -1.22173048, -0.17453293,
    0.        ])]
calculate_camera_robot_transitions(ur3e_arm, ur5e_arm, task_robot_path_configs)

# if __name__ == '__main__':
#     # alpha, beta, gamma set the orientation of the end effector [radians]
#     alpha = -np.pi
#     beta = 0.0
#     gamma = 0

#     # tx, ty, tz set the position of end effector [meter]
#     tx = 0.15
#     ty = -0.35
#     tz = 0.1

#     DH_matrix_UR5e = DH_matrices["ur5e"]
#     DH_matrix_UR3e = DH_matrices["ur3e"]

#     ur_params_ur3e = UR3e_PARAMS(inflation_factor=1)
#     ur_params_ur5e = UR5e_PARAMS(inflation_factor=1)

#     transform_ur5e = Transform(ur_params_ur5e)
#     transform_ur3e = Transform(ur_params_ur3e)

#     bb_ur5e = Building_Blocks(transform=transform_ur5e, ur_params=ur_params_ur5e, env=None, resolution=0.1, p_bias=0.05)
#     bb_ur3e = Building_Blocks(transform=transform_ur3e, ur_params=ur_params_ur3e, env=None, resolution=0.1, p_bias=0.05) # should implement seperate collision functions

#     not_found = True
#     while not_found:
#         try:
#             tx = np.random.uniform(-0.5, 0.5)
#             ty = np.random.uniform(-0.5, 0.5)
#             tz = np.random.uniform(0.1, 0.5)  # Assuming a reasonable height range for the target
#             alpha = np.random.uniform(-np.pi, np.pi)
#             beta = np.random.uniform(-np.pi, np.pi)
#             gamma = np.random.uniform(-np.pi, np.pi)

#             ur5e_arm = ur_kinematics.URKinematics("ur5e")
#             ur3e_arm = ur_kinematics.URKinematics("ur3e")
#             valid_sols = valid_inverse_solutions(tx, ty, tz, alpha, beta, gamma, bb_ur5e, ur5e_arm)
#             print('Valid solutions: ', valid_sols)
#         except Exception as e:
#             print('No solution found!')
