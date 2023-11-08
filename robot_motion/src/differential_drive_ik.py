import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Robot parameters
wheel_radius = 0.05
wheel_separation = 0.15
max_wheel_speed = 0.5  # Adjust max speed if needed

# Simulation parameters
dt = 0.8  # Time step for simulation
total_time = 20.0  # Total time to run the simulation

# Target position
x_target = 1.0
y_target = 1.0

# Initial position and orientation of the robot
x_current = 0.0
y_current = 0.0
theta_current = 0.0

# Create figure and axis objects
fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.grid(True)
ax.set_title('Movement of a Mobile Robot')
ax.set_xlabel('X position (m)')
ax.set_ylabel('Y position (m)')

# Plot target position
ax.plot(x_target, y_target, 'ro', label='Target')

# Circle object to represent the robot
robot_circle = plt.Circle((x_current, y_current), wheel_separation / 2, color='blue', fill=False)
ax.add_artist(robot_circle)

# Animation update function
def update(frame):
    global x_current, y_current, theta_current

    # Calculate angle to target relative to world frame
    angle_to_target = np.arctan2(y_target - y_current, x_target - x_current)

    # Calculate the angle difference
    angle_diff = angle_to_target - theta_current

    # Normalize the angle difference to be within [-pi, pi]
    angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

    # Rotate the robot towards the target if needed
    if abs(angle_diff) > 0.1:  # Allow a small error margin
        angular_speed = np.sign(angle_diff) * max_wheel_speed
        left_wheel_speed = -angular_speed
        right_wheel_speed = angular_speed
    else:
        # Move robot forward
        left_wheel_speed = max_wheel_speed
        right_wheel_speed = max_wheel_speed

    # Update robot's position and orientation
    v = (left_wheel_speed + right_wheel_speed) * wheel_radius / 2
    omega = (right_wheel_speed - left_wheel_speed) * wheel_radius / wheel_separation

    dx = v * np.cos(theta_current) * dt
    dy = v * np.sin(theta_current) * dt
    dtheta = omega * dt

    x_current += dx
    y_current += dy
    theta_current += dtheta

    # Update the robot's representation on the plot
    robot_circle.set_center((x_current, y_current))

    return robot_circle,

# Create animation
ani = FuncAnimation(fig, update, frames=np.arange(0, total_time, dt), blit=False, interval=100)

plt.show()
