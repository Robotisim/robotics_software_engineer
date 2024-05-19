cmake_minimum_required(VERSION 3.5)
project(module_2_assignment)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)





add_executable(back_forth src/turtle_log_spiral.cpp)
ament_target_dependencies(back_forth rclcpp std_msgs geometry_msgs nav_msgs tf2_ros)

add_executable(spiral_turtle src/spiral_turtle.cpp)
ament_target_dependencies(spiral_turtle rclcpp std_msgs geometry_msgs)

install(TARGETS
  back_forth
  spiral_turtle
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})



ament_package()