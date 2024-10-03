# cmake_minimum_required(VERSION 3.8)
# project(maze_solving)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)

# # for PCL libraries
# find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# # for PCAP
# find_package(Pcap REQUIRED)
# include_directories(${PCAP_INCLUDE_DIRS})

# add_executable(mazesolver src/maze_solving_sub.cpp)
# ament_target_dependencies(mazesolver rclcpp geometry_msgs sensor_msgs)
# target_link_libraries(mazesolver ${PCAP_LIBRARIES})

# add_executable(linefollowing src/line_following.cpp)
# ament_target_dependencies(linefollowing rclcpp geometry_msgs sensor_msgs cv_bridge OpenCV)
# target_link_libraries(linefollowing ${OpenCV_LIBS})

# # add_executable(point_cloud_segmentation src/point_cloud_segmentation.cpp)
# # ament_target_dependencies(point_cloud_segmentation rclcpp)
# # target_link_libraries(point_cloud_segmentation ${PCL_LIBRARIES})

# install(TARGETS
#   mazesolver
#   linefollowing
#   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch worlds
#   DESTINATION share/${PROJECT_NAME})

# install(DIRECTORY urdf
#   DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
