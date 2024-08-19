import numpy as np
from kinematicsUtils import balanced_config_autocomplete
bias = (0,2.5)
# task_path = [[0.3, -1.059, -1.229, -0.897, 1.571, 1.571] ,
#             [0.3, -1.059, -1.229, -0.897, 1.571, 1.571] ,
#             [-0.406, -1.349, -1.227, -0.609, 1.571, 1.571] ,
#             [-0.619, -2.458, -1.248, 0.521, 1.571, 1.571] ,
#             [0.237, -1.844, -1.927, 0.586, 1.571, 1.571] ,
#             [0.3, -1.059, -1.229, -0.897, 1.571, 1.571] ,
#         ]

task_path = [
                [-1.254, -0.182, -1.686, -1.134, 2.728, 1.675],
                [-1.254, -0.182, -1.686, -1.134, 2.728, 1.675],
                [0.1, -0.188, -1.712, -1.178, 1.438, 1.571],
                [-1.561, -1.446, -2.057, 0.367, 1.571, 1.571],
                [-1.542, -2.684, -0.751, 0.318, 1.579, 1.585]
]

def pos_sign(x):
    if x >= 0:
        return 1
    return -1

new_path = [balanced_config_autocomplete(c, joint_4_direction=c[4],bias=bias) for c in task_path]
print("task_path = [", end="")
for config in new_path:
    print([round(p,3)for p in config ],",",end="\n\t")
print("]")

