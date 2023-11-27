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
#include "ajgar_perception/octomapSrv.h"
#include <iostream>


ros::Publisher uncommon_points_pub;

bool add(ajgar_perception::octomapSrv::Request &req, ajgar_perception::octomapSrv::Response &res)
{
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(req.kinectInputPt, *cloud2);
    pcl::fromROSMsg(req.maskInputPt, *cloud1);
     
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

    ROS_INFO("Checkpoint 1");
    
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
            // std::cout << cloudpoint << '\n' ; 
        }
    }

    ROS_INFO("Checkpoint 2");
    
    sensor_msgs::PointCloud2 uncommon_points_ros;

    ROS_INFO("Checkpoint 3");
    
    pcl::toROSMsg(uncommon_points, uncommon_points_ros);
    // std::cout << uncommon_points <<'\n' ;
    // std::cout << uncommon_points_ros << '\n' ;

    ROS_INFO("Checkpoint 4");
    
    // Set the frame ID to "camera_depth_optical_frame"
    uncommon_points_ros.header.frame_id = "camera_depth_optical_frame";
    
    ROS_INFO("Checkpoint 5");
    
   //  uncommon_points_pub.publish(uncommon_points_ros);

    // ROS_INFO("Checkpoint 6");
    
    std::cout << "Data Processed " << std::endl;
    std::cout << "Data Published to client " << std::endl;
    res.outputPt = uncommon_points_ros ;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomapSrvNode");

    ros::NodeHandle nh;

    uncommon_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/uncommon_points_topic", 1);
    ros::ServiceServer service = nh.advertiseService("octomapSrv", add);

    ROS_INFO("Server side ready, waiting for request ");
    ros::spin();

    return 0;
}
