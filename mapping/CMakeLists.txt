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

add_executable(oc_grid_pub src/occupancy_grid_pub.cpp)
ament_target_dependencies(oc_grid_pub rclcpp nav_msgs)

add_executable(lidar_to_grid src/lidar_to_grid.cpp)
ament_target_dependencies(lidar_to_grid rclcpp nav_msgs sensor_msgs)

install(TARGETS oc_grid_pub
lidar_to_grid
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds map
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
