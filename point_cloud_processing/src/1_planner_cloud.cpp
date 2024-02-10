#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ> cloud ;

    cloud.push_back(pcl::PointXYZ(1.0 ,2.0 ,3.0));
    cloud.push_back(pcl::PointXYZ(4.0 ,5.0 ,6.0));
    cloud.push_back(pcl::PointXYZ(7.0 ,8.0 ,9.0));
    cloud.push_back(pcl::PointXYZ(10.0,11.0,12.0));
    cloud.push_back(pcl::PointXYZ(12.0,14.0,-15.0));

    std::string path = "/home/luqman/ros2_ws/src/ROS2-Point-Cloud-Clustering-and-Segmentation-for-Autonomous-Behaviour/point_cloud_processing/point_clouds/plane_.pcd";
    pcl::io::savePCDFileASCII(path,cloud);
    std::cout<<cloud.size();


    return 0;

}