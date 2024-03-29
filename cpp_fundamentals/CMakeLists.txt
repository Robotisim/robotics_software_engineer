cmake_minimum_required(VERSION 3.10)
project(LineFollowingRobot)


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)


add_executable(lib_lf src/lib_main.cpp src/lib_line_following.cpp)
target_link_libraries(lib_lf ${OpenCV_LIBS} ${Boost_LIBRARIES})



add_executable(class_r       src/class_robot.cpp )
add_executable(function_sr   src/functions_sensor_reading.cpp )
add_executable(namespaces_rp src/namespaces_sensor_processing.cpp )
add_executable(pointer_r     src/pointers_robot_location.cpp )
add_executable(template_r    src/template_robot.cpp )


