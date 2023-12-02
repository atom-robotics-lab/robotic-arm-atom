#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/String.h>
#include <pcl/common/common.h>

ros::Publisher normals_pub;

// Define point and normal types
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> NormalCloudType;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg)
{
    pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>);
    NormalCloudType::Ptr normals(new NormalCloudType);
    std::vector<std::string> orientations;

    pcl::fromROSMsg(*cloud1_msg, *cloud1);

    // Compute the center point of cloud1
    pcl::PointXYZ center_point;
    pcl::computeCentroid(*cloud1, center_point);

    // Create a kd-tree index and search object for nearest neighbors search
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(cloud1);

    // Find the index of the nearest point to the center point in cloud1
    int nearest_point_index = -1;
    float distance_to_center = std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud1->size(); ++i) {
        float distance = std::sqrt(std::pow((*cloud1)[i].x - center_point.x, 2) +
                                   std::pow((*cloud1)[i].y - center_point.y, 2));
        if (distance < distance_to_center) {
            distance_to_center = distance;
            nearest_point_index = static_cast<int>(i);
        }
    }

    // Check if the nearest point was found
    if (nearest_point_index != -1) {
        // Create a vector to hold the indices of the nearest points to the centroid
        std::vector<int> point_indices;

        // Perform a nearest neighbor search and find the 10 nearest points to the centroid
        std::vector<int> point_indices_temp(1);
        std::vector<float> point_squared_distances(1);
        for (size_t i = 0; i < cloud1->size(); ++i) {
            if (tree->nearestKSearch(cloud1->points[i], 1, point_indices_temp, point_squared_distances) > 0 &&
                std::find(point_indices.begin(), point_indices.end(), point_indices_temp[0]) == point_indices.end()) {
                point_indices.push_back(point_indices_temp[0]);
            }
            if (point_indices.size() == 10) {
                break;
            }
        }

        // Remove points that are further away from the cent
        // // Filter the cloud using a radius outlier removal filter 
        // pcl::RadiusOutlierRemoval<PointType> radius_outlier_removal; 
        // radius_outlier_removal.setInputCloud(cloud1); 
        // radius_outlier_removal.setRadiusSearch(1); // Use a suitable radius for your case 
        // radius_outlier_removal.setMinNeighborsInRadius(1); 
        // radius_outlier_removal.filter(*cloud1);
            // Compute the normals for the remaining points
    pcl::NormalEstimation<PointType, NormalType> normal_estimation;
        normal_estimation.setInputCloud(cloud1);
        normal_estimation.setSearchMethod(tree);
        normal_estimation.setRadiusSearch(0.14); // Use a suitable radius for your case
        normal_estimation.compute(*normals);

    // Find the normal of the centroid
    for (size_t i = 0; i < normals->size(); ++i) {
        if (normals->points[i].normal_x == 0 &&
            normals->points[i].normal_y == 0 &&
            normals->points[i].normal_z == 0) {
            continue;
        }
        Eigen::Vector3f centroid_normal(normals->points[i].normal_x,
                                        normals->points[i].normal_y,
                                        normals->points[i].normal_z);
        // centroid_normal.normalize();
        std::cout << "Normal of the centroid: " << centroid_normal.transpose() << std::endl;
        break;
        }
    }

    sensor_msgs::PointCloud2 normals_msg;
    pcl::toROSMsg(*normals, normals_msg);
    normals_pub.publish(normals_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "normals_node"); ros::NodeHandle nh;
    normals_pub = nh.advertise<sensor_msgs::PointCloud2>("normals", 1);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/mask", 1, pointCloudCallback);
    ros::spin();
    return 0;
}