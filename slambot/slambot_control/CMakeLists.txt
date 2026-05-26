cmake_minimum_required(VERSION 3.8)
project(slambot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)

find_path(LIBSERIAL_INCLUDE_DIR
  NAMES libserial/SerialPort.h
)
find_library(LIBSERIAL_LIBRARY
  NAMES serial
)

if(LIBSERIAL_INCLUDE_DIR AND LIBSERIAL_LIBRARY)
  add_library(diffbot_system_hardware SHARED
    src/diffbot_system.cpp
  )

  target_include_directories(diffbot_system_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${LIBSERIAL_INCLUDE_DIR}
  )

  ament_target_dependencies(diffbot_system_hardware PUBLIC
      "hardware_interface"
      "pluginlib"
      "rclcpp"
      "ros2_control"
  )

  target_link_libraries(diffbot_system_hardware PRIVATE
    ${LIBSERIAL_LIBRARY}
  )

  install(TARGETS
    diffbot_system_hardware
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  pluginlib_export_plugin_description_file(hardware_interface diffbot_system_hardware.xml)

  install(FILES
    diffbot_system_hardware.xml
    DESTINATION share/${PROJECT_NAME}
  )
else()
  message(WARNING "libserial-dev was not found; skipping DiffBotSystemHardware. Install libserial-dev to build hardware support.")
endif()

# Install the launch, urdf, and config directories
install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
