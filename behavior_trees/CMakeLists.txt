cmake_minimum_required(VERSION 3.8)
project(behavior_trees)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(turtlesim REQUIRED)

# NOn ROS2 Basic Understanding trees
add_executable(1a_Sequence_nodes src/sequence_nodes.cpp)
target_link_libraries(1a_Sequence_nodes /usr/local/lib/libbehaviortree_cpp.so)

add_executable(1b_blackboard_ports src/blackboard_ports.cpp)
target_link_libraries(1b_blackboard_ports /usr/local/lib/libbehaviortree_cpp.so)

add_executable(2_drone_ports_fallbacks src/drone_fire_extinguish.cpp)
target_link_libraries(2_drone_ports_fallbacks /usr/local/lib/libbehaviortree_cpp.so)

## ROS2 based TBSIM Driving BT

add_executable(1_sequence_loop src/sequence_loop.cpp)
ament_target_dependencies(1_sequence_loop rclcpp geometry_msgs)
target_link_libraries(1_sequence_loop /usr/local/lib/libbehaviortree_cpp.so)


add_executable(2_goToGoal src/path_follow_fallbacks.cpp)
ament_target_dependencies(2_goToGoal rclcpp turtlesim geometry_msgs)
target_link_libraries(2_goToGoal /usr/local/lib/libbehaviortree_cpp.so)

install(TARGETS 1_sequence_loop 2_goToGoal

DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY trees
# launch worlds models map config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
