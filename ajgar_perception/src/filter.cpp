#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/segment_differences.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

ros::Publisher uncommon_points_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud1_msg, const sensor_msgs::PointCloud2ConstPtr &cloud2_msg)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*cloud1_msg, *cloud1);
    pcl::fromROSMsg(*cloud2_msg, *cloud2);

    // VoxelGrid Downsampling pointcloud
    pcl::VoxelGrid<pcl::PointXYZ> down_sample1;
    down_sample1.setInputCloud(cloud2);
    down_sample1.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_down(new pcl::PointCloud<pcl::PointXYZ>);
    down_sample1.filter(*cloud2_down);

    // VoxelGrid Downsampling mask
    pcl::VoxelGrid<pcl::PointXYZ> down_sample2;
    down_sample2.setInputCloud(cloud1);
    down_sample2.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_down(new pcl::PointCloud<pcl::PointXYZ>);
    down_sample2.filter(*cloud1_down);

    ROS_INFO("Difference");
    pcl::PointCloud<pcl::PointXYZ> uncommon_points;
    for (const pcl::PointXYZ &cloudpoint : cloud2_down->points)
    {
        bool is_common = false;
        for (const pcl::PointXYZ &mask_point : cloud1_down->points)
        {
            if (abs(cloudpoint.x - mask_point.x) <= 0.01 && abs(cloudpoint.y - mask_point.y) <= 0.01)
            {
                is_common = true;
                break;
            }
        }
        if (!is_common)
        {
            uncommon_points.push_back(cloudpoint);
        }
    }
    sensor_msgs::PointCloud2 uncommon_points_ros;
    pcl::toROSMsg(uncommon_points, uncommon_points_ros);
    // Set the frame ID to "camera_depth_optical_frame"
    uncommon_points_ros.header.frame_id = "camera_depth_optical_frame";
    // uncommon_points_pub.publish(uncommon_points_ros);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_segment_differences_node");
    ROS_INFO("Node started");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub(nh, "/mask", 10000);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub(nh, "/kinect/depth/points", 10000);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(100), cloud1_sub, cloud2_sub);
    sync.registerCallback(boost::bind(&pointCloudCallback, _1, _2));

    uncommon_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/uncommon_points_topic", 1);

    ros::spin();
}
