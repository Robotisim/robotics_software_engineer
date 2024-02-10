cmake_minimum_required(VERSION 3.8)
project(point_cloud_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pcd_import src/0_table_scene.cpp)
target_link_libraries(pcd_import ${PCL_LIBRARIES})


add_executable(plane_cloud src/1_planner_cloud.cpp)
target_link_libraries(plane_cloud ${PCL_LIBRARIES})

add_executable(circular_cloud src/2_circular_cloud.cpp)
target_link_libraries(circular_cloud ${PCL_LIBRARIES})

add_executable(filtering_segmentation src/3_filtering_segmentation.cpp)
target_link_libraries(filtering_segmentation ${PCL_LIBRARIES})

add_executable(path_creation src/4_point_path_drawing.cpp)
target_link_libraries(path_creation ${PCL_LIBRARIES})

add_executable(kitti_voxel src/5_kitti_data_voxel.cpp)
ament_target_dependencies(kitti_voxel rclcpp sensor_msgs pcl_conversions)
target_link_libraries(kitti_voxel ${PCL_LIBRARIES})

add_executable(kd_tree src/6_kd_tree.cpp)
# ament_target_dependencies(kd_tree rclcpp sensor_msgs pcl_conversions)
target_link_libraries(kd_tree ${PCL_LIBRARIES})


add_executable(traffic_segmentation src/7_traffic_segmentation.cpp)
ament_target_dependencies(traffic_segmentation rclcpp sensor_msgs pcl_conversions visualization_msgs)
target_link_libraries(traffic_segmentation ${PCL_LIBRARIES})

install(TARGETS
  pcd_import plane_cloud circular_cloud filtering_segmentation  path_creation kitti_voxel kd_tree traffic_segmentation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

