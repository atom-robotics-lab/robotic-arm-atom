#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher centroid_pub;

// Define point type
typedef pcl::PointXYZ PointType;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& mask_msg)
{
    pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>);
    
    // Convert the ROS message to a PCL point cloud
    pcl::fromROSMsg(*mask_msg, *cloud1);

    // Compute the centroid of the point cloud
    pcl::PointXYZ centroid_point;
    pcl::computeCentroid(*cloud1, centroid_point);

    // Create a new point cloud containing only the centroid
    pcl::PointCloud<PointType>::Ptr centroid_cloud(new pcl::PointCloud<PointType>());
    centroid_cloud->points.push_back(centroid_point);

    // Publish the centroid point cloud
    sensor_msgs::PointCloud2 centroid_msg;
    pcl::toROSMsg(*centroid_cloud, centroid_msg);
    centroid_msg.header = mask_msg->header;
    centroid_pub.publish(centroid_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "centroid_node");
    ros::NodeHandle nh;

    centroid_pub = nh.advertise<sensor_msgs::PointCloud2>("/centroid_cloud", 1);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/mask", 1, pointCloudCallback);
    
    ros::spin();
    
    return 0;
}