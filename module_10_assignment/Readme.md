# These assignment-Tasks are not yet Finilized


### Assignments for Module #10 : Point Clouds
- Create all files in *module_10_assignment* package
### Assignment 1: Efficient Point Cloud Segmentation and Analysis Using Octrees
**Task**:
Load a 3D LiDAR scan from the KITTI dataset.
Implement or utilize an existing Octree library to construct an Octree from the point cloud data. Ensure your implementation supports dynamic point cloud data insertion and efficient spatial querying.
- Feature Extraction

    Using the Octree, implement a method to segment the point cloud into distinct regions or objects. Focus on identifying features critical to autonomous driving, such as road boundaries, vehicles, pedestrians, and other obstacles.
    Extract geometric features from each segment, such as size, position, and orientation, which could be useful for navigation and obstacle avoidance.
    Visualization and Analysis

Visualize the segmented point cloud and extracted features using a tool like Rviz or PCLVisualizer. Highlight different segments with unique colors or markers to distinguish between them.
Analyze the efficiency of using the Octree structure for these tasks, particularly in terms of computational speed and memory usage compared to traditional point cloud processing methods.