#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <fstream>


typedef pcl::PointXYZ PointT;

void cloud_saver(const std::string& file_name,std::string& path, pcl::PointCloud<PointT>::Ptr cloud_arg){
    pcl::PCDWriter cloud_writer;
    cloud_writer.write<PointT>(path+std::string(file_name),*cloud_arg);
}

int main()
{
    // ********************************     Reading the Cloud
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>) ;
    pcl::PCDReader cloud_reader;
    std::string path = "/home/luqman/ros2_ws/src/ROS2-Point-Cloud-Clustering-and-Segmentation-for-Autonomous-Behaviour/point_cloud_processing/point_clouds/";
    std::string input_cloud = "tb3_world.pcd";
    cloud_reader.read(path+input_cloud , *cloud);

    // ********************************     Voxel Filter

    pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>) ;
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05 , 0.05, 0.05);
    voxel_filter.filter(*voxel_cloud);

    // ********************************     Pass through Filter
    // Along X Axis
    pcl::PointCloud<PointT>::Ptr passthrough_cloud (new pcl::PointCloud<PointT>) ;
    pcl::PassThrough<PointT> passing_x;
    passing_x.setInputCloud(voxel_cloud);
    passing_x.setFilterFieldName("x");
    passing_x.setFilterLimits(-1.7,1.7);
    passing_x.filter(*passthrough_cloud);

    // Along Y Axis
    pcl::PassThrough<PointT> passing_y;
    passing_y.setInputCloud(passthrough_cloud);
    passing_y.setFilterFieldName("y");
    passing_y.setFilterLimits(-1.7,1.7);
    passing_y.filter(*passthrough_cloud);

    // ********************************     Planner Segmentation
    pcl::PointIndices::Ptr  inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr  coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<PointT>::Ptr plane_segmented_cloud (new pcl::PointCloud<PointT>) ;
    pcl::SACSegmentation<PointT> plane_segmentor;
    pcl::ExtractIndices<PointT> indices_extractor;

    plane_segmentor.setInputCloud(passthrough_cloud);
    plane_segmentor.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentor.setMethodType(pcl::SAC_RANSAC);
    plane_segmentor.setDistanceThreshold(0.01);
    plane_segmentor.segment(*inliers,*coefficients);

    indices_extractor.setInputCloud(passthrough_cloud);
    indices_extractor.setIndices(inliers);
    indices_extractor.setNegative(false);
    indices_extractor.filter(*plane_segmented_cloud);
    cloud_saver("plane.pcd",path,plane_segmented_cloud);

    // ********************************     Cylinder Segmentation
    // Normal Extraction Objects
    pcl::ModelCoefficients::Ptr       cylinder_co    (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr            cylinder_in    (new pcl::PointIndices);
    pcl::search::KdTree<PointT>::Ptr  tree           (new pcl::search::KdTree<PointT> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals  (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr      cylinder_cloud (new pcl::PointCloud<PointT> ());
    // Normals computation objects
    pcl::NormalEstimation<PointT,pcl::Normal>            normals_estimator;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> cylinder_segmentor;
    pcl::ExtractIndices<PointT>                          cylinder_indices_extractor;
    pcl::ExtractIndices<pcl::Normal>                     cylinder_normal_indices_extractor;
// Performing estimation of normals
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(plane_segmented_cloud);
    normals_estimator.setKSearch(30);
    normals_estimator.compute(*cloud_normals);

    // Parameters for segmentation
    cylinder_segmentor.setOptimizeCoefficients(true);
	cylinder_segmentor.setModelType(pcl::SACMODEL_CYLINDER);
	cylinder_segmentor.setMethodType(pcl::SAC_RANSAC);
	cylinder_segmentor.setNormalDistanceWeight(0.5);
	cylinder_segmentor.setMaxIterations(10000);
	cylinder_segmentor.setDistanceThreshold(0.05);
	cylinder_segmentor.setRadiusLimits(0.1, 0.4);
  int looping_var=0;

    while(true){


    // Appplying segmentation
    cylinder_segmentor.setInputCloud(plane_segmented_cloud);
	cylinder_segmentor.setInputNormals(cloud_normals);
	cylinder_segmentor.segment(*cylinder_in,*cylinder_co);

    // extracting indices
    cylinder_indices_extractor.setInputCloud(plane_segmented_cloud);
    cylinder_indices_extractor.setIndices(cylinder_in);
    cylinder_indices_extractor.setNegative(false);
    cylinder_indices_extractor.filter(*cylinder_cloud);

    if(!cylinder_cloud->points.empty()){
        std::stringstream loop_name_cloud; loop_name_cloud <<"cloud_"<<looping_var<<".pcd";
        std::cout<<"Cloud Contains " <<cylinder_cloud->points.size()<<std::endl;
        if(cylinder_cloud->points.size() > 50){
          cloud_saver(loop_name_cloud.str(),path,cylinder_cloud);
            looping_var++;
        }

        cylinder_indices_extractor.setNegative(true);
        cylinder_indices_extractor.filter(*plane_segmented_cloud);

        // processing normals
        cylinder_normal_indices_extractor.setInputCloud(cloud_normals);
        cylinder_normal_indices_extractor.setIndices(cylinder_in);
        cylinder_normal_indices_extractor.setNegative(true);
        cylinder_normal_indices_extractor.filter(*cloud_normals);

    }
    else{
        return 0;
    }


    }
  return 0;
}