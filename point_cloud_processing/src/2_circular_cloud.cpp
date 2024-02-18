#include <iostream>
#include <filesystem>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud ;
    double radius = 3.0;
    int num_points= 50;
    double angular_step_size = 2.0* M_PI /num_points;

    for(int i=0;i<num_points;i++){
    pcl::PointXYZRGB point;
    double angle = i *angular_step_size;
    point.x= radius * std::cos(angle);
    point.y= radius * std::sin(angle);
    point.z= 1.0;

    point.r= 255*std::cos(angle);
    point.g= 255*std::sin(angle);
    point.b= 255*std::cos(angle +M_PI_2);

    cloud.push_back(point);
    }


    std::string pcd_file_name = "circular_cloud.pcd";
	std::filesystem::path ros2_ws_path    = std::filesystem::current_path();
    std::filesystem::path point_cloud_dir = ros2_ws_path/"src/ROS2-Point-Cloud-Clustering-and-Segmentation-for-Autonomous-Behaviour/point_cloud_processing/point_clouds/";
    std::string pcd_file_path             = point_cloud_dir/pcd_file_name;

    pcl::io::savePCDFileASCII(pcd_file_path,cloud);



    return 0;

}