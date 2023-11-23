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
    ne.setRadiusSearch(0.5);
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

int main()
{
    // Example usage
    PointCloud::Ptr cloud1(new PointCloud);
    PointCloud::Ptr cloud2(new PointCloud);

    // Add a single point to cloud2
    PointType single_point;
    single_point.x = 0.0;
    single_point.y = 0.0;
    single_point.z = 0.0;
    cloud2->push_back(single_point);#centroid

    // Fill in sample point cloud data for testing in cloud1
    for (float x = -1.0; x <= 1.0; x += 0.1) {
        for (float y = -1.0; y <= 1.0; y += 0.1) {
            PointType point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            cloud1->push_back(point);
        }
    }

    // Calculate normals and get orientations as strings
    std::vector<std::string> normal_orientations = calculateNormals(cloud1, cloud2);

    // Display the calculated orientations
    for (size_t i = 0; i < normal_orientations.size(); ++i) {
        std::cout << "Point " << i << " Normal Orientation: " << normal_orientations[i] << std::endl;
    }

    return 0;
}
