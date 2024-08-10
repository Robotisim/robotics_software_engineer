cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# enable_testing()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Adding for c_xy
find_package(Eigen3 REQUIRED)

add_executable(linear_kf src/linear_kf.cpp)
ament_target_dependencies(linear_kf rclcpp std_msgs nav_msgs visualization_msgs )
target_include_directories(linear_kf PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(linear_kf Eigen3::Eigen)



# Add library for EKF
add_library(ekf_lib src/ekf_lib.cpp)
target_include_directories(ekf_lib PRIVATE ${EIGEN3_INCLUDE_DIR} include)
target_link_libraries(ekf_lib Eigen3::Eigen)

# Add executable for EKF node
add_executable(ekf_node src/ekf_node.cpp)
target_include_directories(ekf_node PRIVATE include)
target_link_libraries(ekf_node ekf_lib)
ament_target_dependencies(ekf_node rclcpp sensor_msgs visualization_msgs)


install(TARGETS
  linear_kf
  ekf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
 DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # ament_add_gtest(test_kalman_filter test/kalman_test.cpp)
  # target_link_libraries(test_kalman_filter kalman_lib "-fsanitize=address,undefined" Eigen3::Eigen)
  # target_include_directories(test_kalman_filter PRIVATE include)
  # target_compile_options(test_kalman_filter PRIVATE
  # "-fsanitize=address,undefined"
  # "-g")

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
