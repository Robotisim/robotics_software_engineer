cmake_minimum_required(VERSION 3.8)
project(behavior_trees)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(nav_msgs REQUIRED)

# Non ROS2 BT examples
add_executable(a_bt_sequence_nodes src/bt_sequence_nodes.cpp)
target_link_libraries(a_bt_sequence_nodes /usr/local/lib/libbehaviortree_cpp.so)

add_executable(b_bt_blackboard_ports_fallback src/bt_blackboard_ports_fallback.cpp)
target_link_libraries(b_bt_blackboard_ports_fallback /usr/local/lib/libbehaviortree_cpp.so)



# ROS2 Bt Node
add_executable(a_r2_bt_seq src/r2_bt_sequence.cpp)
target_link_libraries(a_r2_bt_seq /usr/local/lib/libbehaviortree_cpp.so)
ament_target_dependencies(a_r2_bt_seq rclcpp std_msgs geometry_msgs)

add_executable(b_r2_bt_go_to_goal src/r2_bt_go_to_goal.cpp)
ament_target_dependencies(b_r2_bt_go_to_goal rclcpp turtlesim geometry_msgs)
target_link_libraries(b_r2_bt_go_to_goal /usr/local/lib/libbehaviortree_cpp.so)



install(TARGETS
  a_bt_sequence_nodes b_bt_blackboard_ports_fallback
  a_r2_bt_seq b_r2_bt_go_to_goal
  DESTINATION lib/${PROJECT_NAME})


  install(DIRECTORY trees
  # launch worlds models map config
    DESTINATION share/${PROJECT_NAME}/
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
