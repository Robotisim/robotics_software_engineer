cmake_minimum_required(VERSION 3.8)
project(drive_mobile_robot)

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


add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(drive_turtle src/drive_turtle.cpp)
ament_target_dependencies(drive_turtle rclcpp geometry_msgs)

install(TARGETS
  talker listener drive_turtle
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()