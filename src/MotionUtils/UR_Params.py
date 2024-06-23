import numpy as np
import math


class UR5e_PARAMS(object):
    '''
    UR5e_PARAMS determines physical costants for UR5e Robot manipulator
    @param inflation_factor - by what factor inflate the minimal sphere radious of each link
    @param tool_length - the lenght of the tool [meters]. for the gripper, set 0.135 meter
    '''
    def __init__(self, inflation_factor=1.0, tool_length=0.135):
                    # alpha, a, d, theta_const
        self.ur_DH = [ [0, 0, 0.1625, 0], # shoulder - > base_link
                    [np.pi/2, 0,0, 0], #upper_arm_link - > shoulder
                    [0, -0.425, 0, 0], # forearm_link -> upper_arm_link
                    [0, -0.3922, 0.133, 0], # wrist1 -> forearm_link
                    [np.pi/2, 0, 0.0997, 0], # wrist2 -> wrist1
                    [-np.pi/2, 0, 0.0996, 0]] # wrist3 -> wrist2

        self.ur_geometry = [['shoulder_link', np.array([0,0, -0.1625]), 'z'],
                            ['upper_arm_link',   np.array([-0.425,0,0.135]), 'x'],
                            ['upper_arm_link',   np.array([0,0,0.135]), 'z'],
                            ['forearm_link'  ,   np.array([0,0,0.135]),'z'],
                            ['forearm_link'  ,   np.array([-0.392,0,0.015]),'x'],
                            ['wrist_1_link',     np.array([0,0,-0.109]), 'z'],
                            ['wrist_1_link',     np.array([0,-0.11,0]), 'y'],
                            ['wrist_1_link',     np.array([0,0.05,0]), 'y'],
                            ['wrist_2_link',     np.array([0,0.0825,0]), 'y'],
                            ['wrist_2_link',     np.array([0,-0.05,0]), 'y'],
                            ['wrist_3_link',     np.array([0,0, tool_length]), 'z']]

        self.ur_links = [
                            'shoulder_link',
                            'upper_arm_link',
                            'forearm_link',
                            'wrist_1_link',
                            'wrist_2_link',
                            'wrist_3_link'
                        ]

        self.ur_links_color = {
                                'base_link':'gray',
                                'base_link_inertia':'gray',
                                'shoulder_link':'gray',
                                'upper_arm_link':'yellow',
                                'forearm_link':'green',
                                'wrist_1_link':'purple',
                                'wrist_2_link':'black',
                                'wrist_3_link':'blue'
                             }

        self.mechanical_limits = {
                                    'shoulder_link':[-np.pi, np.pi],
                                    'upper_arm_link':[-np.pi, np.pi],
                                    'forearm_link':[-np.pi, np.pi],
                                    'wrist_1_link':[-np.pi, np.pi],
                                    'wrist_2_link':[-np.pi, np.pi],
                                    'wrist_3_link':[-np.pi, np.pi]
                                }

        self.min_sphere_radius = {
                                    'shoulder_link':0.06,
                                    'upper_arm_link':0.05,
                                    'forearm_link':0.05,
                                    'wrist_1_link':0.04,
                                    'wrist_2_link':0.04,
                                    'wrist_3_link':0.04
                                }

        self.sphere_radius = {key: val * inflation_factor for key, val in self.min_sphere_radius.items()}



class UR3e_PARAMS(object):
    '''
    UR3e_PARAMS determines physical costants for UR3e Robot manipulator
    @param inflation_factor - by what factor inflate the minimal sphere radious of each link
    @param tool_length - the lenght of the tool [meters]. for the gripper, set 0.135 meter
    '''
    def __init__(self, inflation_factor=1.0, tool_length=0.135):
        # alpha, a, d, theta_const
        self.ur_DH = [
                        [0, 0, 0.1519, 0],           # shoulder -> base_link
                        [-np.pi/2, 0, 0, 0],         # upper_arm_link -> shoulder
                        [0, -0.24365, 0, 0],         # forearm_link -> upper_arm_link
                        [0, -0.21325, 0.11235, 0],   # wrist1 -> forearm_link
                        [np.pi/2, 0, 0.08535, 0],    # wrist2 -> wrist1
                        [-np.pi/2, 0, 0.0819 + tool_length, 0]  # wrist3 -> wrist2, including tool length
                    ]

        self.ur_geometry = [
            ['shoulder_link', np.array([0, 0, -0.1519]), 'z'],
            ['upper_arm_link', np.array([-0.24365, 0, 0]), 'x'],
            ['upper_arm_link', np.array([0, 0, 0.11235]), 'z'],
            ['forearm_link', np.array([0, 0, 0.11235]), 'z'],
            ['forearm_link', np.array([-0.21325, 0, 0.015]), 'x'],
            ['wrist_1_link', np.array([0, 0, -0.109]), 'z'],
            ['wrist_1_link', np.array([0, -0.11, 0]), 'y'],
            ['wrist_1_link', np.array([0, 0.05, 0]), 'y'],
            ['wrist_2_link', np.array([0, 0.0825, 0]), 'y'],
            ['wrist_2_link', np.array([0, -0.05, 0]), 'y'],
            ['wrist_3_link', np.array([0, 0, tool_length]), 'z']
        ]


        self.ur_links = [ 'shoulder_link', 'upper_arm_link',
                        'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']

        self.ur_links_color = {'base_link':'gray', 'base_link_inertia':'gray',
                               'shoulder_link':'gray','upper_arm_link':'yellow',
                             'forearm_link':'green', 'wrist_1_link':'purple'
                            , 'wrist_2_link':'black', 'wrist_3_link':'blue'}

        self.mechanical_limits = {
            'shoulder_link': [-np.pi, np.pi],
            'upper_arm_link': [-np.pi, np.pi],
            'forearm_link': [-np.pi, np.pi],
            'wrist_1_link': [-np.pi, np.pi],
            'wrist_2_link': [-np.pi, np.pi],
            'wrist_3_link': [-np.pi, np.pi]
        }

        # self.min_sphere_radius = {
        #     'shoulder_link': 0.03,  # Adjusted from 0.06 to 0.03
        #     'upper_arm_link': 0.025,  # Adjusted from 0.05 to 0.025
        #     'forearm_link': 0.025,  # Adjusted from 0.05 to 0.025
        #     'wrist_1_link': 0.02,  # Adjusted from 0.04 to 0.02
        #     'wrist_2_link': 0.02,  # Adjusted from 0.04 to 0.02
        #     'wrist_3_link': 0.02  # Adjusted from 0.04 to 0.02
        # }

        self.min_sphere_radius = {
            'shoulder_link': 0.02,  # Further tightened from 0.03 to 0.02
            'upper_arm_link': 0.015,  # Further tightened from 0.025 to 0.015
            'forearm_link': 0.015,  # Further tightened from 0.025 to 0.015
            'wrist_1_link': 0.01,  # Further tightened from 0.02 to 0.01
            'wrist_2_link': 0.01,  # Further tightened from 0.02 to 0.01
            'wrist_3_link': 0.01  # Further tightened from 0.02 to 0.01
        }

        self.sphere_radius = {key: val * inflation_factor for key, val in self.min_sphere_radius.items()}


class Transform(object):
    '''
    Trasform class implemets the Forward kinematics method
    to finds the coordinates of the spheres along the manipulator's links
    '''
    def __init__(self, ur_params):
        self.ur = ur_params.ur_DH
        self.frame_list = ur_params.ur_links
        self.sphere_radius = ur_params.sphere_radius
        self.robot_geometry = ur_params.ur_geometry
        self.local_sphere_coords = dict()
        axis_dict = {'x':0, 'y':1, 'z':2}

        for frame in self.frame_list:
            self.local_sphere_coords[frame] = []

        for frame, offset, axis in self.robot_geometry:
            spheres_amount =  max(int(math.ceil(abs(offset[axis_dict[axis]] / (self.sphere_radius[frame])))+1) , 3)
            local_sphere_offset = np.linspace(0, offset[axis_dict[axis]], num=spheres_amount, endpoint=True)
            for sphere_offset in local_sphere_offset:
                if axis == 'x':
                    self.local_sphere_coords[frame].append(np.array([sphere_offset, offset[1],offset[2],1], dtype= float))
                elif axis =='y':
                    self.local_sphere_coords[frame].append(np.array([offset[0],sphere_offset,offset[2],1], dtype=float))
                else:
                    self.local_sphere_coords[frame].append(np.array([offset[0],offset[1],sphere_offset ,1], dtype=float))

    def dh_transform(self, alpha, a, d, theta):
        # return np.array([[np.cos(theta), -np.sin(theta), 0, a],
        #                 [np.sin(theta) * np.cos(alpha) , np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
        #                 [np.sin(theta) * np.sin(alpha) , np.cos(theta) * np.sin(alpha), np.cos(alpha),  d * np.cos(alpha)],
        #                 [0,0,0,1]], dtype=float)
        return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]], dtype=float)

    def get_trans_matrix(self, conf):
        '''
        Returns the transformation matrix for given configuration
        '''
        trans = [[] for _ in range(len(self.ur))]
        for i in range(len(self.ur)):
            alpha, a, d, theta_const =  self.ur[i]
            theta = conf[i] + theta_const
            trans[i] = np.asarray([[np.cos(theta), -np.sin(theta), 0, a],
                                [np.sin(theta) * np.cos(alpha) , np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
                                [np.sin(theta) * np.sin(alpha) , np.cos(theta) * np.sin(alpha), np.cos(alpha),  d * np.cos(alpha)],
                                [0,0,0,1]], dtype="object") #dtype=float

        trans_matrix = dict()
        trans_matrix['shoulder_link'] = trans[0]
        for i in range(1, len(trans)):
            trans_matrix[self.frame_list[i]] = np.matmul(trans_matrix[self.frame_list[i-1]] , trans[i])

        return trans_matrix

    def get_global_sphere_coords(self, trans_matrix):
        '''
        Returns the coordinates of the spheres along the manipulator for a given transformation matriix
        '''
        global_sphere_coords = dict()
        for frame in self.local_sphere_coords.keys():
            global_sphere_coords[frame] = []
            for i in range(len(self.local_sphere_coords[frame])):
                current_global_coords = np.matmul(trans_matrix[frame], self.local_sphere_coords[frame][i].T)
                global_sphere_coords[frame].append(current_global_coords)
        return global_sphere_coords

    def conf2sphere_coords(self, conf):
        '''
        Returns the coordinates of the spheres along the manipulator's links for a given configuration,
        in the base_link frame
        @param conf - some configuration
        '''
        trans_matrix = self.get_trans_matrix(conf)
        return self.get_global_sphere_coords(trans_matrix)
