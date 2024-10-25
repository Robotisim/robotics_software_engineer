cmake_minimum_required(VERSION 3.8)
project(module_5_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../../extern/eigen)
find_package(Eigen3 REQUIRED)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(angles REQUIRED)



# LQR Library
add_library(lqr_lib STATIC src/lqr_lib.cpp)
target_include_directories(lqr_lib PRIVATE ${EIGEN_INCLUDE_DIR} include)
target_link_libraries(lqr_lib Eigen3::Eigen)

# PID library
add_library(pid_lib STATIC src/pid_lib/pid_lib.cpp)
target_link_libraries(pid_lib)
ament_target_dependencies(pid_lib rclcpp)
target_include_directories(pid_lib PRIVATE include)

# Executable for LQR node test
add_executable(lqr_nodetst src/lqr_node.cpp)
ament_target_dependencies(lqr_nodetst rclcpp nav_msgs geometry_msgs tf2 angles visualization_msgs)
target_link_libraries(lqr_nodetst lqr_lib Eigen3::Eigen)
target_include_directories(lqr_nodetst PRIVATE ${EIGEN_INCLUDE_DIR} include)

# Executable for line following with camera
add_executable(lineFollowing src/linefollowingwithcamerakip.cpp)
ament_target_dependencies(lineFollowing rclcpp sensor_msgs cv_bridge geometry_msgs)
target_link_libraries(lineFollowing ${OpenCV_LIBS})

add_executable(line_following_pid src/line_following/line_following_main.cpp src/line_following/line_following.cpp )
ament_target_dependencies(line_following_pid rclcpp geometry_msgs sensor_msgs cv_bridge)
target_link_libraries(line_following_pid ${OpenCV_LIBS} pid_lib)
target_include_directories(line_following_pid PRIVATE include)

# Install targets
install(TARGETS
        lqr_lib
        lqr_nodetst
        lineFollowing
        line_following_pid
        pid_lib
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds 
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
