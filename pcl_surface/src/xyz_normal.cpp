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
#include <geometry_msgs/Vector3.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>

ros::Publisher normals_pub;
ros::Publisher marker_pub;
ros::Publisher centroid_pub;

// Define point and normal types
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> NormalCloudType;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg)
{
    pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>);
    NormalCloudType::Ptr normals(new NormalCloudType);
    pcl::fromROSMsg(*cloud1_msg, *cloud1);

    //add extremitities

     // Calculate the average x and y coordinates
    float avg_x = 0.0;
    float avg_y = 0.0;

    for (const auto& point : *cloud1) {
        avg_x += point.x;
        avg_y += point.y;
    }

    size_t num_points = cloud1->size();
    if (num_points > 0) {
        avg_x /= num_points;
        avg_y /= num_points;

        // Find a point with similar x and y coordinates
        PointType centroid_point;
        

        // Search for a point with x and y values close to the averages
        for (const pcl::PointXYZ& point : cloud1->points) {
            
            if (std::abs(point.x - avg_x) < 0.001 && std::abs(point.y - avg_y) < 0.001) {
                centroid_point = point;
                
                break;
                
            }
        }
        printf("%f , %f , %f\n",centroid_point.x,centroid_point.y,centroid_point.z );

        // Now centroid_point contains a point with similar x and y coordinates to the average

        // Create a new point cloud and push the centroid point into it
        pcl::PointCloud<pcl::PointXYZ> centroid_cloud;
        centroid_cloud.push_back(centroid_point);

        size_t centroid_index = 0;
        for (size_t i = 0; i < cloud1->size(); ++i) {
            const PointType& cloud1_point = cloud1->points[i];
            if (std::abs(cloud1_point.x - centroid_point.x) < 0.001 &&
                std::abs(cloud1_point.y - centroid_point.y) < 0.001 &&
                std::abs(cloud1_point.z - centroid_point.z) < 0.001) {
                centroid_index = i;
                break;
            }
        }

    // Create a new point cloud containing only the center point


    pcl::NormalEstimation<PointType, NormalType> normal_estimation;
    normal_estimation.setInputCloud(cloud1);

    // Set the search method using a kd-tree
    // pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    // tree->setInputCloud(cloud1);
    // normal_estimation.setSearchMethod(tree);

    // Set the radius for finding neighbors
    normal_estimation.setRadiusSearch(0.09); // Use a suitable radius for your case

    // Compute the normals
    normal_estimation.compute(*normals);

    // Access the normal at the center point
    Eigen::Vector3f center_normal(
        normals->points[centroid_index].normal_x,
        normals->points[centroid_index].normal_y,
        normals->points[centroid_index].normal_z
    );

    // Normalize the normal vector
    center_normal.normalize();

    std::cout << "Normal of the point at the center: " << center_normal.transpose() << std::endl;

    // Create a ROS message for the normal vector
    geometry_msgs::Vector3 normal_msg;
    normal_msg.x = center_normal[0];
    normal_msg.y = center_normal[1];
    normal_msg.z = center_normal[2];


    sensor_msgs::PointCloud2 centroid_msg;
    pcl::toROSMsg(centroid_cloud, centroid_msg);
    
    centroid_msg.header = cloud1_msg->header;


    centroid_msg.header.frame_id="camera_depth_optical_frame";
    printf("%s\n", cloud1_msg->header.frame_id.c_str());


    centroid_pub.publish(centroid_msg);
    // Publish the normal vector
    normals_pub.publish(normal_msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "normals_node"); ros::NodeHandle nh;
    normals_pub = nh.advertise<geometry_msgs::Vector3>("/normals", 1);
    centroid_pub = nh.advertise<sensor_msgs::PointCloud2>("/centroid_cloud", 1);

    
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/mask", 1, pointCloudCallback);
    ros::spin();
    return 0;
}
