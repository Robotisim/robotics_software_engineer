#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <filesystem>

int main(){
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCDReader cloud_reader;

    std::string pcd_file_name = "tb3_world.pcd";
	std::filesystem::path ros2_ws_path    = std::filesystem::current_path();
    std::filesystem::path point_cloud_dir = ros2_ws_path/"src/ROS2-Point-Cloud-Clustering-and-Segmentation-for-Autonomous-Behaviour/point_cloud_processing/point_clouds/";
    std::string pcd_file_path             = point_cloud_dir/pcd_file_name;

    cloud_reader.read(pcd_file_path,*cloud);

    std::cout<<"Number of point "<< cloud->width * cloud->height << std::endl;

    return 0;
}