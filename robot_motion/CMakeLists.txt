cmake_minimum_required(VERSION 3.10)

project(robot_motion)

include_directories(include)


add_executable(${PROJECT_NAME} main.cpp src/camera.cpp src/lidar.cpp src/motors.cpp )

