# ROS 2 Jazzy / Gazebo Sim Migration Status

Branch: `jazzy-harmonic-migration`

## Build command

Use this capped command on the host so the machine does not get overloaded:

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --symlink-install --parallel-workers 12 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

The explicit Python executable avoids the local `/home/luqman/.local/bin/python3.11`
environment, which is missing `catkin_pkg`.

## Verified so far

- Full workspace builds on ROS 2 Jazzy with the capped build command.
- All 39 launch files pass `ros2 launch <file> --show-args`.
- Modern Gazebo Sim runtime checks were run headless with 24 second timeouts:
  - `custom_robots/launch/gazebo.launch.py`: clean startup, `/clock`, `/cmd_vel`,
    `/odom`, `/tf` bridges, robot spawn succeeds.
  - `robot_sensing/launch/custom_sensors.launch.py`: clean startup, `/clock`,
    `/cmd_vel`, `/odom`, `/tf`, `/scan`, `/camera/image_raw` bridges, robot spawn
    succeeds.
  - `robot_sensing/launch/camera_line_following.launch.py`: clean startup after
    installing package model assets and replacing stale absolute mesh paths.
  - `robot_sensing/launch/lidar_maze_solving.launch.py`: clean startup.
  - `mapping/launch/maze_tb3_bringup.launch.py`: clean startup using Gazebo Sim.
  - `control_systems/launch/velocity_controller.launch.py`: clean Gazebo Sim +
    `gz_ros2_control` startup.
  - `control_systems/launch/goal_planer.launch.py`: clean startup using Gazebo Sim.
  - `control_systems/launch/lqr_multi_goals.launch.py`: clean startup using Gazebo Sim.
  - `slambot_control/launch/mock_controller_bringup.launch.py`: controller manager,
    `mock_components/GenericSystem`, `diff_drive_controller`, and
    `joint_state_broadcaster` start. The host prints the normal non-realtime desktop
    warning: FIFO RT scheduling is not permitted.

## Remaining blockers

- `libserial-dev` is not installed on this host, so the real ESP32 serial hardware plugin
  is skipped at build time. Install `libserial-dev` to compile `DiffBotSystemHardware`.
- `rtabmap` and `rtabmap_ros` are not installed, so 3D mapping runtime is syntax-checked
  but cannot be fully exercised on this host yet. `rosdep check` reports:
  `ros-jazzy-rtabmap`, `ros-jazzy-rtabmap-ros`, `ros-jazzy-rtabmap-slam`, and
  `ros-jazzy-rtabmap-viz`.
- The real hardware launch is syntax-checked only on this host. Runtime needs
  `libserial-dev`, the compiled serial hardware plugin, and the actual controller device.
- Nav2 params should be tested at runtime after the simulation stack is fully migrated.
