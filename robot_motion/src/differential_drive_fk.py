import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# Robot parameters
wheel_radius = 0.05  # radius of the wheels
wheel_separation = 0.15  # distance between the two wheels

# Time parameters
dt = 0.5  # time step

# Speeds
right_wheel_speed = 2.0  # speed of the left wheel
left_wheel_speed = 1.0  # speed of the right wheel

# Initialize the robot's position and orientation
x_position = 0.0
y_position = 0.0
orientation = 0.0

# Create figure and axis objects
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
trajectory_line, = ax.plot([], [], label='Robot Trajectory')  # Line object to update trajectory
robot_circle = plt.Circle((x_position, y_position), wheel_separation/2, color='blue', fill=False)  # Circle object to represent the robot
ax.add_artist(robot_circle)

# Set up the plot limits
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.grid(True)
ax.set_title('Forward Kinematics of a Mobile Robot')
ax.set_xlabel('X position (m)')
ax.set_ylabel('Y position (m)')
ax.legend()

# Button to advance the simulation
ax_button = plt.axes([0.7, 0.05, 0.1, 0.075])
button = Button(ax_button, 'Step')

# This function will be called when the button is clicked
def step(event):
    global x_position, y_position, orientation
    v = wheel_radius * (right_wheel_speed + left_wheel_speed) / 2.0
    omega = wheel_radius * (right_wheel_speed - left_wheel_speed) / wheel_separation

    delta_x = v * np.cos(orientation) * dt
    delta_y = v * np.sin(orientation) * dt
    delta_theta = omega * dt

    x_position += delta_x
    y_position += delta_y
    orientation += delta_theta

    # Update the trajectory line
    trajectory_line.set_xdata(np.append(trajectory_line.get_xdata(), x_position))
    trajectory_line.set_ydata(np.append(trajectory_line.get_ydata(), y_position))

    # Update the robot's position
    robot_circle.center = (x_position, y_position)
    ax.draw_artist(robot_circle)
    fig.canvas.blit(ax.bbox)

button.on_clicked(step)

plt.show()
