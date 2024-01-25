cmake_minimum_required(VERSION 3.8)
project(drive_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(p1_a_drive_node src/p1_a_drive_node.cpp)
ament_target_dependencies(p1_a_drive_node rclcpp geometry_msgs)


add_executable(p1_b_tb3_follower src/p1_b_tb3_follower.cpp)
ament_target_dependencies(p1_b_tb3_follower rclcpp sensor_msgs cv_bridge)
target_link_libraries(p1_b_tb3_follower ${OpenCV_LIBS})




install(TARGETS p1_a_drive_node p1_b_tb3_follower

  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
