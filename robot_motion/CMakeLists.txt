cmake_minimum_required(VERSION 3.10)
project(RobotMotionProject)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include header files from the 'include' directory
include_directories(include)

# Add the main program
add_executable(robot_motion main.cpp src/RobotMotion.cpp src/Sensor.cpp)

# If you have additional libraries or dependencies, link them here
# For example, if you had a math library to link, you would uncomment the following line:
# target_link_libraries(robot_motion m)
