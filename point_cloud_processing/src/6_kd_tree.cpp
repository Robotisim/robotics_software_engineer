#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <iostream>
int main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < 100; i++)
  {
    pcl::PointXYZ p;
    p.x = static_cast<float> (rand()) / static_cast<float> (RAND_MAX);
    p.y = static_cast<float> (rand()) / static_cast<float> (RAND_MAX);
    p.z = static_cast<float> (rand()) / static_cast<float> (RAND_MAX);
    cloud->push_back(p);
  }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);


    // Create a query point

    pcl::PointXYZ searchPoint;
    searchPoint.x = 0.6f;
    searchPoint.y = 0.6f;
    searchPoint.z = 0.6f;

    int K=5;
    std::vector<int> indices(K);
    std::vector<float> distances(K);
    kdtree.nearestKSearch(searchPoint, K, indices, distances);

    for (int i=0; i<indices.size(); i++)
    {
        std::cout << "indices[" << i << "]:" << indices[i] << " distances[" << i << "]:" << distances[i] << std::endl;
    }



    return 0;
}