cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TwoInts.msg"
  "msg/CameraLidar.msg"
  DEPENDENCIES sensor_msgs
)

# uncomment below after first build to build nodes
  #- On first colcon build Message libraries will be generated

# find_package(custom_interfaces REQUIRED)

# add_executable(combine_ints src/combine_ints.cpp)
# ament_target_dependencies(combine_ints rclcpp custom_interfaces)

# add_executable(combine_sensors src/combine_sensors.cpp)
# ament_target_dependencies(combine_sensors rclcpp custom_interfaces)

# install(TARGETS
#   combine_ints
#   combine_sensors
#   DESTINATION lib/${PROJECT_NAME}
# )



ament_package()
