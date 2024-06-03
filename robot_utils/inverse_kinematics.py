import numpy as np
from ur_ikfast import ur_kinematics
from UR_Params import UR5e_PARAMS, UR3e_PARAMS, Transform
from building_blocks import Building_Blocks

# Define the tool length and DH matrices for different UR arms
tool_length = 0.135  # [m]

DH_matrices = {
    "UR3e": np.matrix([[0, np.pi / 2.0, 0.15185],
                       [-0.24355, 0, 0],
                       [-0.2132, 0, 0],
                       [0, np.pi / 2.0, 0.13105],
                       [0, -np.pi / 2.0, 0.08535],
                       [0, 0, 0.0921 + tool_length]]),
    "UR5e": np.matrix([[0, np.pi / 2.0, 0.1625],
                       [-0.425, 0, 0],
                       [-0.3922, 0, 0],
                       [0, np.pi / 2.0, 0.1333],
                       [0, -np.pi / 2.0, 0.0997],
                       [0, 0, 0.0996 + tool_length]])
}

def valid_inverse_solutions(tx, ty, tz, alpha, beta, gamma, bb, ur_arm):
    """
    Get valid inverse kinematics solutions that are collision-free.

    Computes the inverse kinematics solutions for a given target end-effector
    position and orientation, ensuring that the resulting joint configurations
    are collision-free.

    :param tx: Target x position of the end-effector.
    :param ty: Target y position of the end-effector.
    :param tz: Target z position of the end-effector.
    :param alpha: Target roll angle (orientation) of the end-effector.
    :param beta: Target pitch angle (orientation) of the end-effector.
    :param gamma: Target yaw angle (orientation) of the end-effector.
    :param bb: Building_Blocks object for collision checking.
    :param ur_arm: Specific UR arm object providing inverse kinematics capabilities.
    :return: List of valid joint angle solutions that achieve the specified end-effector
             pose and are collision-free.
    """
    target_pose = [tx, ty, tz, alpha, beta, gamma]  # Updated to use alpha, beta, gamma directly
    q_solutions = ur_arm.inverse(target_pose)
    
    valid_solutions = []
    for q in q_solutions:
        if not bb.is_in_collision(q):
            valid_solutions.append(q)
    return valid_solutions


def forward_kinematic_solution(joint_angles, ur_arm):
    """
    Compute the forward kinematics.
    
    :param joint_angles: List of joint angles.
    :param ur_arm: Specific UR arm object.
    :return: End effector pose.
    """
    return ur_arm.forward(joint_angles)

if __name__ == '__main__':
    # alpha, beta, gamma set the orientation of the end effector [radians]
    alpha = -np.pi
    beta = 0.0
    gamma = 0
    
    # tx, ty, tz set the position of end effector [meter]
    tx = 0.15
    ty = -0.35
    tz = 0.1  # the height to drop a cube, for instance

    ur_type = "UR5e"  # Change this to "UR3e" or other types if needed
    DH_matrix = DH_matrices[ur_type]
    
    if ur_type == "UR3e":
        ur_params = UR3e_PARAMS(inflation_factor=1)
    elif ur_type == "UR5e":
        ur_params = UR5e_PARAMS(inflation_factor=1)
    else:
        raise ValueError(f"Unsupported UR arm type: {ur_type}")
    
    transform = Transform(ur_params)
    bb = Building_Blocks(transform=transform, ur_params=ur_params, env=None, resolution=0.1, p_bias=0.05)
    not_found = True
    while not_found:
        try:
            tx = np.random.uniform(-0.5, 0.5)
            ty = np.random.uniform(-0.5, 0.5)
            tz = np.random.uniform(0.1, 0.5)  # Assuming a reasonable height range for the target
            alpha = np.random.uniform(-np.pi, np.pi)
            beta = np.random.uniform(-np.pi, np.pi)
            gamma = np.random.uniform(-np.pi, np.pi)
            
            ur_arm = ur_kinematics.URKinematics(ur_type.lower())
            valid_sols = valid_inverse_solutions(tx, ty, tz, alpha, beta, gamma, bb, ur_arm)
            not_found = False
            print('Valid solutions: ', valid_sols)
        except Exception as e:
            print('No solution found:', e)
