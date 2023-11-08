cmake_minimum_required(VERSION 3.8)
project(drive_turtle)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(p1_single_straight_drive src/p1_a_single_straight_drive.cpp)
ament_target_dependencies(p1_single_straight_drive rclcpp turtlesim geometry_msgs)


add_executable(p1_multi_straight_drive src/p1_b_multi_straight_drive.cpp)
ament_target_dependencies(p1_multi_straight_drive rclcpp turtlesim geometry_msgs)

add_executable(p1_multi_turtle_controller src/p1_c_multi_turtle_controller.cpp)
ament_target_dependencies(p1_multi_turtle_controller rclcpp turtlesim geometry_msgs)



install(TARGETS
  p1_multi_turtle_controller p1_single_straight_drive p1_multi_straight_drive
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
