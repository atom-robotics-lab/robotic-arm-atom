#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>  // Include for pcl::toROSMsg
#include <array> // Include for std::array
#include <sstream> // Include for std::stringstream

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<NormalType> NormalCloud;


extern "C" {

    std::vector<std::string> calculateNormals(const std::array<std::array<float,3>,N>& points1, const std::array<std::array<float,3>,M>& points2)
    {
        // Create a container for the normals
        NormalCloud::Ptr normals(new NormalCloud);

        // Create PointCloud instances from the input arrays
        PointCloud::Ptr cloud1(new PointCloud);
        PointCloud::Ptr cloud2(new PointCloud);

    for (size_t i = 0; i < points1.size(); ++i) {
        const std::array<float, 3>& point = points1[i];
        PointType p; 
        p.x = point[0];
        p.y = point[1];
        p.z = point[2];
        cloud1->push_back(p);
    }

    for (size_t i = 0; i < points2.size(); ++i) {
        const std::array<float, 3>& point = points2[i];
        PointType p;
        p.x = point[0];
        p.y = point[1];
        p.z = point[2];
        cloud2->push_back(p);
    }

        // Convert PointCloud instances to PointCloud2 messages
        sensor_msgs::PointCloud2::Ptr cloud1_msg(new sensor_msgs::PointCloud2);
        sensor_msgs::PointCloud2::Ptr cloud2_msg(new sensor_msgs::PointCloud2);

        pcl::toROSMsg(*cloud1, *cloud1_msg);
        pcl::toROSMsg(*cloud2, *cloud2_msg);


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
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "normals");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Add the rest of your code here, e.g., subscribing to PointCloud2 messages, calling calculateNormals, etc.

    return 0;
}