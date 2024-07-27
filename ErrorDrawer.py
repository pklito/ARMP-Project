import matplotlib.pyplot as plt
import re
import numpy as np
from matplotlib.animation import FuncAnimation
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
plate_width, plate_height = 0.21, 0.297

filtered_errors = [(t, np.clip(-x + plate_width/2, 0, plate_width), np.clip(y + plate_height/2,0,plate_height)) for t, x, y, z in zip(normalized_timestamps, error_x, error_y, error_z) if z == 0.035]

normalized_timestamps, ball_positions_x, ball_positions_y = zip(*filtered_errors)

FRAMERATE = 30
print("Length of timestamps:", len(normalized_timestamps))
print("Length of ball_positions_x:", len(ball_positions_x))
print("Length of ball_positions_y:", len(ball_positions_y))
print("total time of simulation:", normalized_timestamps[-1] - normalized_timestamps[0], "simulated time: ", len(normalized_timestamps)/FRAMERATE)

fig, ax = plt.subplots()
ax.set_xlim(0, plate_width)
ax.set_ylim(0, plate_height)
ball_dot, = plt.plot([], [], 'ro', markersize=23)
old_dots, = plt.plot([],[], 'x', markersize=8)

plt.grid(True)

def init():
    ball_dot.set_data([], [])
    return old_dots, ball_dot

def update(frame):
    if frame < len(ball_positions_x):
        ball_dot.set_data([ball_positions_x[frame]], [ball_positions_y[frame]])
        old_dots.set_data(ball_positions_x[max(frame-10,0):max(frame-1,0)], ball_positions_y[max(frame-10,0):max(frame-1,0)])
    return old_dots, ball_dot

try:
    anim = FuncAnimation(fig, update, frames=len(normalized_timestamps), init_func=init, blit=True)
    anim.save('ball_simulation4.gif', writer='pillow', fps=30)
except Exception as e:
    print(f"Error saving GIF: {e}")
