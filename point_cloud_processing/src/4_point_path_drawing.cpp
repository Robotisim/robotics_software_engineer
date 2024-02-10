#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>
#include <iostream>

int main() {
    std::string pcd_file_name = "plane.pcd";
	std::filesystem::path ros2_ws_path    = std::filesystem::current_path();
    std::filesystem::path point_cloud_dir = ros2_ws_path/"src/ROS2-Point-Cloud-Clustering-and-Segmentation-for-Autonomous-Behaviour/point_cloud_processing/point_clouds/";
    std::string pcd_file_path             = point_cloud_dir/pcd_file_name;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud);

    pcl::visualization::PCLVisualizer viewer("Point Cloud with Path");
    viewer.addPointCloud(cloud,"Groudn Plane ");
    float min_x,min_y,max_y =0;
        for(int i=0;i<cloud->points.size();i++){
            pcl::PointXYZ point = cloud->points[i];
        if(point.x < min_x){
            min_x=point.x;
        }
        if(point.y < min_y){
            min_y = point.y;
        }
        if (point.y > max_y) {
            max_y = point.y;
        }
        }

    float ball_radius = 0.2;
    float ball_x = min_x + 1.2;
    float ball_y = min_y;

    int i=0;
    while(ball_y <= max_y){
    pcl::PointXYZ point;
    point.x =ball_x;
    point.y=ball_y;


    viewer.addSphere(point,0.1,0.0,1.0,0.0,"Sphere"+std::to_string(i++));
    ball_y += ball_radius * 1.5;

    }
    viewer.spin();


    return 0;
}
