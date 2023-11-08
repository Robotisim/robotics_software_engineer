cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(astar_node
  src/main.cpp
  src/Astar.cpp
  src/Dijkstra.cpp
)

target_link_libraries(astar_node ${OpenCV_LIBRARIES} )
ament_target_dependencies(astar_node rclcpp )


add_executable(rrt_node
  src/main_rrt.cpp
  src/rrt.cpp
)

target_link_libraries(rrt_node ${OpenCV_LIBRARIES} )
ament_target_dependencies(rrt_node rclcpp )




install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}

)


install (TARGETS
  astar_node
  rrt_node
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
