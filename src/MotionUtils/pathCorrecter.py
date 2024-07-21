import numpy as np

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

new_path = [[c[0],c[1],c[2],-(c[1]+c[2]) + pos_sign(c[4])*np.deg2rad(bias[1]) - np.pi,pos_sign(c[4])*np.pi/2, np.pi/2 + np.deg2rad(bias[0])] for c in task_path]
print("task_path = [", end="")
for config in new_path:
    print([round(p,3)for p in config ],",",end="\n\t")
print("]")

