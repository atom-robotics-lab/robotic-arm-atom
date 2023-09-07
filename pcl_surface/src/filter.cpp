#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher filtered_cloud_pub;

void syncedCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud1,
                         const sensor_msgs::PointCloud2ConstPtr& input_cloud2)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*input_cloud1, *pcl_cloud1);
    pcl::fromROSMsg(*input_cloud2, *pcl_cloud2);
    ROS_INFO("DATA");

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(pcl_cloud1);
    voxel_grid.setLeafSize(0.01, 0.01, 0.01);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*filtered_cloud1);

    voxel_grid.setInputCloud(pcl_cloud2);
    voxel_grid.setLeafSize(0.01, 0.01, 0.01);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*filtered_cloud2);

    ROS_INFO("Process");

    sensor_msgs::PointCloud2 filtered_ros_cloud;
    pcl::toROSMsg(*filtered_cloud2, filtered_ros_cloud);

    filtered_cloud_pub.publish(filtered_ros_cloud);
    ROS_INFO("PUB");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_synced_callbacks_node");
    ROS_INFO("NODE STARTED");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub1(nh, "/mask", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub2(nh, "/mask", 1);
    
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cloud_sub1, cloud_sub2);
    sync.registerCallback(boost::bind(&syncedCloudCallback, _1, _2));

    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_combined_cloud", 1);

    ros::spin();

    return 0;
}
