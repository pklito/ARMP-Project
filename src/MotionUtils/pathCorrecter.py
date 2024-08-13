import numpy as np
from kinematicsUtils import balanced_config_autocomplete
bias = (0,2.5)
task_path = [[0.3, -1.059, -1.229, -0.897, 1.571, 1.571] ,
            [0.3, -1.059, -1.229, -0.897, 1.571, 1.571] ,
            [-0.406, -1.349, -1.227, -0.609, 1.571, 1.571] ,
            [-0.619, -2.458, -1.248, 0.521, 1.571, 1.571] ,
            [0.237, -1.844, -1.927, 0.586, 1.571, 1.571] ,
            [0.3, -1.059, -1.229, -0.897, 1.571, 1.571] ,
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

