#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void pclCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*input, cloud);
    pcl::io::savePLYFileBinary("insta10.ply", cloud);
    ROS_INFO("PointCloud2 data saved as PLY file.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_extractor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, pclCallback);
    
    ros::spin();
    
    return 0;
}
