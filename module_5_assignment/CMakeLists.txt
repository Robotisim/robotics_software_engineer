cmake_minimum_required(VERSION 3.8)
project(module_5_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../../extern/eigen)
find_package(Eigen3 REQUIRED)
# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)



add_library(lqr_lib STATIC /home/kushal16/rse_ws1/robotics_software_engineer/module_5_assignment/src/lqr_lib.cpp)
target_include_directories(lqr_lib PRIVATE ${EIGEN_INCLUDE_DIR} include)
target_link_libraries(lqr_lib Eigen3::Eigen)

# add_executable(lqr_node src/lqr_node.cpp)
# target_link_libraries(lqr_node lqr_lib Eigen3::Eigen)
# ament_target_dependencies(lqr_node rclcpp nav_msgs geometry_msgs tf2 angles)
# target_include_directories(lqr_node PRIVATE ${EIGEN_INCLUDE_DIR} include)


# Add the new lqr_nodetst executable
add_executable(lqr_nodetst /home/kushal16/rse_ws1/robotics_software_engineer/module_5_assignment/src/lqr_node.cpp)
target_link_libraries(lqr_nodetst lqr_lib Eigen3::Eigen)  # Link with the LQR library
ament_target_dependencies(lqr_nodetst rclcpp nav_msgs geometry_msgs tf2 angles visualization_msgs)
target_include_directories(lqr_nodetst PRIVATE ${EIGEN_INCLUDE_DIR} include)

# add_executable(odom_pub src/odom_pub.cpp)
# ament_target_dependencies(odom_pub rclcpp geometry_msgs nav_msgs)

# for line following
add_executable(lineFollowing src/linefollowingwithcamerakip.cpp)
ament_target_dependencies(lineFollowing rclcpp sensor_msgs cv_bridge geometry_msgs)
target_link_libraries(lineFollowing ${OpenCV_LIBS})

install(TARGETS
        lqr_lib
        lqr_nodetest
        lineFollowing
       DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch worlds urdf
  DESTINATION share/${PROJECT_NAME})

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
