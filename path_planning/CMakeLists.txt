cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Occupancy Grid
add_executable(occupancy_grid_node src/occupancy_grid.cpp )
ament_target_dependencies(occupancy_grid_node rclcpp nav_msgs tf2_ros )


# Astar Opencv based
add_executable(astar_ros2_node  src/astar_ros2.cpp )
target_link_libraries(astar_ros2_node ${OpenCV_LIBRARIES} )
ament_target_dependencies(astar_ros2_node rclcpp  geometry_msgs nav_msgs)


# Astar Ros2- Rviz Based
add_executable(astar_opencv_node  src/astar_opencv.cpp )
target_link_libraries(astar_opencv_node ${OpenCV_LIBRARIES} )
ament_target_dependencies(astar_opencv_node rclcpp )


install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}

)


install (TARGETS
  occupancy_grid_node
  astar_ros2_node
  astar_opencv_node
  DESTINATION lib/${PROJECT_NAME}

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
