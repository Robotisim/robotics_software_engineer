cmake_minimum_required(VERSION 3.8)
project(control_systems)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(velocity_controller_drive src/velocity_controller_drive.cpp)
ament_target_dependencies(velocity_controller_drive rclcpp tf2_ros geometry_msgs)

add_executable(goal_linear src/goal_linear.cpp)
ament_target_dependencies(goal_linear rclcpp tf2_ros geometry_msgs nav_msgs)

add_executable(goal_planer src/goal_planer.cpp)
ament_target_dependencies(goal_planer rclcpp tf2_ros geometry_msgs nav_msgs)



install(TARGETS
velocity_controller_drive goal_linear goal_planer
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
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
