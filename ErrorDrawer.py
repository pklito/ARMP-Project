import matplotlib.pyplot as plt
import re
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

from PIL import Image
from matplotlib.animation import PillowWriter
from regex import F
from sklearn.preprocessing import normalize

# def init():
#     ball_dot.set_data([], [])
#     return ball_dot,

# def update(frame):
#     if frame < len(ball_positions_x):
#         ball_dot.set_data(ball_positions_x[frame], ball_positions_y[frame])
#     return ball_dot,

file_path = 'Square_path_log.txt'
log_data = None

with open(file_path, 'r') as file:
    log_data = file.read()

timestamps = []
robot_positions = []
target_positions = []

pattern = re.compile(
    r"(\d+\.\d+):DEBUG:syncrhronized_balancing\.py;{'error': (None|array\(\[.*?\]\)), 'robot_pos': \[(.*?)\], 'target_pos': \[(.*?)\]}")
matches = pattern.findall(log_data)

timestamps = []
errors = []

for match in matches:
    timestamp, error, robot_pos_str, target_pos_str = match
    # print("match: ", match)
    timestamps.append(float(timestamp))
    if error != "None":
        error_cleaned = error.replace('array([', '').replace('])', '').replace(' ', '')
        error_values = np.array([float(x) for x in error_cleaned.split(',')])
        errors.append(error_values)
        # print("error_values: ", error_values)
    else:
        errors.append([0.0, 0.0, 0.0])  # Assuming zero error if not specified


start_time = timestamps[0]
normalized_timestamps = [t - start_time for t in timestamps]

error_x, error_y, error_z = list(zip(*errors))

# Filter errors based on z position
plate_width, plate_height = 0.297, 0.21
PLATE_LEFT = -plate_width/2
PLATE_RIGHT = plate_width/2
PLATE_UP = plate_height/2
PLATE_DOWN = -plate_height/2
PLATE_CENTER = (np.average(PLATE_LEFT,PLATE_RIGHT), np.average(PLATE_DOWN,PLATE_UP))

# flip axis: x = y, y = -x
filtered_errors = [(t, np.clip(y, PLATE_LEFT, PLATE_RIGHT), np.clip(-x, PLATE_DOWN, PLATE_UP)) for t, x, y, z in zip(normalized_timestamps, error_x, error_y, error_z) if z == 0.035]

normalized_timestamps, ball_positions_x, ball_positions_y = zip(*filtered_errors)

FRAMERATE = 30
print("Length of timestamps:", len(normalized_timestamps))
print("Length of ball_positions_x:", len(ball_positions_x))
print("Length of ball_positions_y:", len(ball_positions_y))
print("total time of simulation:", normalized_timestamps[-1] - normalized_timestamps[0], "simulated time: ", len(normalized_timestamps)/FRAMERATE)

####This piece of code tries to ensure the rate of input data matches the gif FPS, however, its close enough as is.
timed_errors = []
lt = normalized_timestamps[0]-1/FRAMERATE
taccum = 0
for t, x, y in filtered_errors:
    if taccum < -1/FRAMERATE:           # Only add a value if time difference is significant enough.
        taccum += 1/FRAMERATE
    elif taccum > 1/FRAMERATE:
        timed_errors.append((t,x,y))
        timed_errors.append((t,x,y))
        taccum += t - lt - 2/FRAMERATE
        lt = t
    else:
        timed_errors.append((t,x,y))
        taccum += t - lt - 1/FRAMERATE
        lt = t

normalized_timestamps, ball_positions_x, ball_positions_y = zip(*timed_errors)
print("Length of timeset timestamps:", len(normalized_timestamps))
print("total time of simulation:", normalized_timestamps[-1] - normalized_timestamps[0], "simulated time: ", len(normalized_timestamps)/FRAMERATE)

#Figure stuff
fig, ax = plt.subplots()
ax.set_aspect(1)
grid_step = 0.05
ax.set_xticks([i for i in np.arange(0,PLATE_RIGHT, grid_step)] + [-i for i in np.arange(grid_step,PLATE_LEFT, grid_step)])
ax.set_yticks([i for i in np.arange(0,PLATE_UP, grid_step)] + [-i for i in np.arange(grid_step, PLATE_DOWN, grid_step)])
ax.set_xlim(PLATE_LEFT,PLATE_RIGHT)
ax.set_ylim(PLATE_DOWN,PLATE_UP)

plt.grid(True, 'major')

#Add squares
# square = patches.Rectangle((0.1,0.25), 0.05, 0.1, linewidth=1, alpha=0.1, facecolor='b',hatch='//')
# ax.add_patch(square)

ball_dot, = plt.plot([], [], 'ro', markersize=23)
old_dots, = plt.plot([],[], 'x', markersize=8)


def init():
    ball_dot.set_data([], [])
    return old_dots, ball_dot

def update(frame):
    if frame < len(ball_positions_x):
        ball_dot.set_data([ball_positions_x[frame]], [ball_positions_y[frame]])
        old_dots.set_data(ball_positions_x[max(frame-10,0):max(frame-1,0)], ball_positions_y[max(frame-10,0):max(frame-1,0)])
    return old_dots, ball_dot

try:
    anim = FuncAnimation(plt.gcf(), update, frames=len(normalized_timestamps), init_func=init, blit=True)
    anim.save('ball_simulation5.gif', writer='pillow', fps=30)
except Exception as e:
    print(f"Error saving GIF: {e}")
