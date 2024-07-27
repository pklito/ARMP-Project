import matplotlib.pyplot as plt
import re
import numpy as np
from matplotlib.animation import FuncAnimation
from PIL import Image
from matplotlib.animation import PillowWriter


file_path = 'Square_path_log.txt'
log_data = None

# Read log data from the file
with open(file_path, 'r') as file:
    log_data = file.read()

# Updated pattern to handle both 'None' and array format in the error field
pattern = re.compile(
    r"(\d+\.\d+):DEBUG:syncrhronized_balancing\.py;{'error': (None|array\(\[(.*?)\]\)), 'robot_pos': \[(.*?)\], 'target_pos': \[(.*?)\]}")
matches = pattern.findall(log_data)

timestamps = []
errors = []

for match in matches:
    timestamp, error, error_values, robot_pos_str, target_pos_str = match
    timestamps.append(float(timestamp))
    # If error is not 'None', extract the values
    if error != "None":
        # Use error_values if present, else error
        error_values = error_values if error_values else error
        # Remove 'array([' and '])' if present
        error_values = error_values.replace('array([', '').replace('])', '')
        errors.append([float(x) for x in error_values.split(', ')])
    else:
        errors.append([0.0, 0.0, 0.0])  # Assuming zero error if not specified

# Normalize timestamps by subtracting the start time
start_time = timestamps[0]
normalized_timestamps = [t - start_time for t in timestamps]

# Extract error components (x, y, z)
error_x, error_y, error_z = list(zip(*errors))

# Plot and animate the error over time
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(0, max(normalized_timestamps))
ax.set_ylim(min(min(error_x), min(error_y), min(error_z)), max(max(error_x), max(error_y), max(error_z)))
ax.set_xlabel('Time Elapsed (s)')
ax.set_ylabel('Error')
ax.set_title('Error vs. Time')
line_x, = ax.plot([], [], label='Error X', color='r')
line_y, = ax.plot([], [], label='Error Y', color='g')
line_z, = ax.plot([], [], label='Error Z', color='b')
ax.legend()

def init():
    line_x.set_data([], [])
    line_y.set_data([], [])
    line_z.set_data([], [])
    return line_x, line_y, line_z

def update(frame):
    line_x.set_data(normalized_timestamps[:frame], error_x[:frame])
    line_y.set_data(normalized_timestamps[:frame], error_y[:frame])
    line_z.set_data(normalized_timestamps[:frame], error_z[:frame])
    return line_x, line_y, line_z

ani = FuncAnimation(fig, update, frames=len(normalized_timestamps), init_func=init, blit=True)

# Save the animation as a GIF
ani.save('error_animation.gif', writer='pillow', fps=30)

plt.show()
