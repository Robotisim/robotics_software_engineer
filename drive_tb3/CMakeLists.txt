cmake_minimum_required(VERSION 3.8)
project(drive_tb3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo_msgs REQUIRED)


add_executable(p2_multi_tb3_sqaure_drive src/p2_multi_tb3_sqaure_drive.cpp)
ament_target_dependencies(p2_multi_tb3_sqaure_drive rclcpp geometry_msgs)

add_executable(p3_a_acceleration src/p3_a_acceleration.cpp)
ament_target_dependencies(p3_a_acceleration rclcpp geometry_msgs)


add_executable(p3_b_angular_velocity src/p3_b_angular_velocity.cpp)
ament_target_dependencies(p3_b_angular_velocity rclcpp geometry_msgs)

add_executable(p3_c_linear_goal src/p3_c_linear_goal.cpp)
ament_target_dependencies(p3_c_linear_goal rclcpp geometry_msgs nav_msgs)

add_executable(p3_d_xy_goal src/p3_d_xy_goal.cpp)
ament_target_dependencies(p3_d_xy_goal rclcpp geometry_msgs nav_msgs tf2)

add_executable(p5_a_lidar_data_sub src/p5_a_lidar_data_sub.cpp)
ament_target_dependencies(p5_a_lidar_data_sub rclcpp geometry_msgs sensor_msgs)

add_executable(p5_b_maze_solving src/p5_b_maze_solving.cpp)
ament_target_dependencies(p5_b_maze_solving rclcpp geometry_msgs sensor_msgs)

add_executable(p5_c_camera_data_sub src/p5_c_camera_data_sub.cpp)
ament_target_dependencies(p5_c_camera_data_sub rclcpp geometry_msgs sensor_msgs cv_bridge)
target_link_libraries(p5_c_camera_data_sub ${OpenCV_LIBS})

add_executable(p5_d_line_following src/p5_d_line_following.cpp)
ament_target_dependencies(p5_d_line_following rclcpp geometry_msgs sensor_msgs cv_bridge)
target_link_libraries(p5_d_line_following ${OpenCV_LIBS})

add_executable(p6_a_occupancy_grid src/p6_a_occupancy_grid.cpp)
ament_target_dependencies(p6_a_occupancy_grid rclcpp geometry_msgs  nav_msgs)

add_executable(p6_b_sdf_spawner src/p6_b_sdf_spawner.cpp)
ament_target_dependencies(p6_b_sdf_spawner rclcpp gazebo_msgs)


install(TARGETS p2_multi_tb3_sqaure_drive
                p3_a_acceleration p3_b_angular_velocity p3_c_linear_goal p3_d_xy_goal
                p5_a_lidar_data_sub p5_b_maze_solving p5_c_camera_data_sub p5_d_line_following
                p6_a_occupancy_grid p6_b_sdf_spawner

  DESTINATION lib/${PROJECT_NAME})


install(PROGRAMS
      src/p6_c_commander_api_go_to_pose.py

DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY launch worlds models map config
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
