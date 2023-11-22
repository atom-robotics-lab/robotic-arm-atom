#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <vector> // Include for std::vector
#include <sstream> // Include for std::stringstream

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<NormalType> NormalCloud;

std::vector<std::string> calculateNormals(const PointCloud::Ptr& cloud1, const PointCloud::Ptr& cloud2)
{
    // Create a container for the normals
    NormalCloud::Ptr normals(new NormalCloud);

    // Create a KD-Tree for cloud1
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(cloud1);

    // Initialize normal estimation object
    pcl::NormalEstimation<PointType, NormalType> ne;
    ne.setInputCloud(cloud2);
    ne.setSearchMethod(tree);

    // Output datasets
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    std::vector<std::string> orientations;

    // Print the orientations of the normals
    for (size_t i = 0; i < normals->size(); ++i) {
        Eigen::Vector3f normal_vector = normals->points[i].getNormalVector3fMap();

        // Convert components to strings and concatenate
        std::stringstream orientation_string;
        orientation_string << "X: " << normal_vector(0) << ", "
                          << "Y: " << normal_vector(1) << ", "
                          << "Z: " << normal_vector(2);

        // Store the orientation as a string in the vector
        orientations.push_back(orientation_string.str());
    }

    return orientations;
}