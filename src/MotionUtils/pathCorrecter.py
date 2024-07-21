import numpy as np

bias = (0,-2.5)
task_path = [[0.3, -1.059, -1.229, -0.874, 1.716, 1.523],
             [-0.406, -1.349, -1.227, -0.598, 1.3, 1.575],
             [-0.619, -2.458, -1.248, 0.568, 1.566, 1.555],
             [0.237, -1.844, -1.927, 0.597, 1.566, 1.555],
             [0.3, -1.059, -1.229, -0.874, 1.716, 1.523]]

new_path = [[c[0],c[1],c[2],-(c[1]+c[2]) + np.deg2rad(bias[1]) - np.pi,-np.pi/2 if c[4] < 0 else np.pi/2, np.pi/2 + np.deg2rad(bias[0])] for c in task_path]
print("task_path = [", end="")
for config in new_path:
    print([round(p,3)for p in config ],",",end="\n\t")
print("]")

