cmake_minimum_required(VERSION 3.8)
project(mapping)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(oc_grid_pub src/occupancy_grid_pub.cpp)
ament_target_dependencies(oc_grid_pub rclcpp nav_msgs)


add_executable(og_sub src/occupancy_grid_sub.cpp)
ament_target_dependencies(og_sub rclcpp nav_msgs)

add_executable(lidar_sub src/lidar_sub.cpp)
ament_target_dependencies(lidar_sub rclcpp nav_msgs sensor_msgs visualization_msgs tf2_ros)


add_executable(marker_to_map src/occupancy_grid_to_map.cpp)
ament_target_dependencies(marker_to_map rclcpp nav_msgs visualization_msgs)

install(TARGETS oc_grid_pub
  lidar_sub og_sub marker_to_map
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds map models config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
