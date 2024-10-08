
# Module # 4 : Sensor Data Processing ( V1.0 )
- Package Created : *robot_sensing*
    - Real World Sensor Data processing
    - Gazebo Simulation Sensor Data Processing
    - ROS2 Bags
---
## Lectures

- `a_module_intro`
- `b_tb3_simulation_sensors_exploration`
    - Echo multiple sensors
    - Understand lidar data
    - subscribe lidar data , Log Things
    - *overlay*
        - [screen] : What Is 2D lidar data
- `c_lidar_data_processing`
    - Echo multiple sensors
    - Understand lidar data
    - subscribe lidar data , Log Things
    - *overlay*
        - [screen] : What we recieve - how to make it control with minimum
- `d_maze_building_robot_positioning`
    - Build Maze
    - Keep Robot spawn location
    - *overlay*
        - [screen] : Gazebo working system and spawning of robot and maze model
        - [face] :
            - Gazebo is intersting software to work with
- `e_maze_solving`
    - Explain different points at which robot will take action, stop at those points
    - *overlay*
        - [screen] : B part -> problems we have -> how we will solve it
            Cases at which we process lidar data
        - [face] : We will not make our robot take actions based on sensor data
- `f_camera_data`
    - Visualization . data understanding , logging , different image sizes
    - *overlay* :
        - [screen] : image sizes , bandwidth effect on size change
        - [face] : Lidar Data is interesting but lets move to more complex data a 3D image which is used for Vision puposes.
- `g_line_gazebo`
    - Blender model designing
    - Gazebo Importing and launching with Robot
    - *overlay* :
        - [screen] : High level explanation , how things are happening
- `h_line_segmentation`
    - *overlay* :
        - [screen] : How canny edge detector works
- `i_boundary_extraction`
    - *overlay* :
        - [screen] : How algo works
- `j_mid_point_extraction`
    - *overlay* :
        - [screen] : How canny algo works
- `k_basic_error_control_algorithm`
    - We will make it smooth in next module
    - *overlay* :
        - [screen] : How algo works
        - [face] : Lets make our robot move , using contol algorithm a propoertional - in upcoming module we will have detail control algroithm understanding
- `l_rosbags`
    - Recording sensor data

- `m_sensors_into_custom_robots`
    - lidar,camera
- `n_depth_camera`
    - We will make it smooth in next module
    - *overlay* :
        - [screen] : How algo works
        - [face] : Lets make our robot move , using contol algorithm a propoertional - in upcoming module we will have detail control algroithm understanding
---
## Formating and linting
- clang-format -i  f_formate.cpp
- clang-tidy f_formate.cpp --fix
---