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
#include <visualization_msgs/Marker.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/extract_indices.h>
#include <boost/make_shared.hpp>

ros::Publisher uncommon_points_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg, const sensor_msgs::PointCloud2ConstPtr& cloud2_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*cloud1_msg, *cloud1);
    pcl::fromROSMsg(*cloud2_msg, *cloud2);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud2);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    ne.setRadiusSearch(0.03);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    ROS_INFO("Difference");


    pcl::Normal centroid_noentroid_pointrmal;

    pcl::PointCloud<pcl::PointXYZ> uncommon_points;
for (const pcl::PointXYZ& cloudpoint : cloud2->points) {

    for (const pcl::PointXYZ& cen_point : cloud1->points) {
        if (abs(cloudpoint.x-cen_point.x)<=0.01 && abs(cloudpoint.y-cen_point.y)<=0.01 && abs(cloudpoint.z-cen_point.z)<=0.01) {
            
            centroid_normalentroid_point = normals->at(cloud1);
            break;
        }
    }

}



    
entroid_point
        // Create a marker for the normal at the centroid
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_depth_optical_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "normals";
        marker.id = 0;  // Unique ID for the centroid normal marker
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // Set the position of the marker at the centroid point
        geometry_msgs::Point point;
        point.x = cloud1->points[cloud1].x;
        point.y = cloud1->points[cloud1].y;
        point.z = cloud1->points[cloud1].z;
        marker.points.push_back(point);

        // Set the orientation of the marker to represent the normal
        geometry_msgs::Vector3 normal_vec;
        normal_vec.x = centroid_normal.normal_x;
        normal_vec.y = centroid_normal.normal_y;
        normal_vec.z = centroid_normal.normal_z;
        marker.pose.orientation.x = normal_vec.x;
        marker.pose.orientation.y = normal_vec.y;
        marker.pose.orientation.z = normal_vec.z;
        marker.pose.orientation.w = 1.0;

        // Set marker scale and color
        marker.scale.x = 0.02;  // Arrow shaft diameter
        marker.scale.y = 0.03;  // Arrow head diameter
        marker.scale.z = 0.1;   // Arrow head length
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // Publish the marker
        uncommon_points_pub.publish(marker);
    }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "centroid_normal_node");
    ROS_INFO("Node started");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub(nh, "/centroid", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub(nh, "/kinect/depth/points", 1);

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cloud1_sub, cloud2_sub);
    sync.registerCallback(boost::bind(&pointCloudCallback, _1, _2));
    
    uncommon_points_pub = nh.advertise<visualization_msgs::Marker>("/centroid_normal_marker", 1);

    ros::spin();

    return 0;
}
        // Set the orientation of the marker to represent the normal
        geometry_msgs::Vector3 normal_vec;
        normal_vec.x = centroid_normal.normal_x;
        normal_vec.y = centroid_normal.normal_y;
        normal_vec.z = centroid_normal.normal_z;
        marker.pose.orientation.x = normal_vec.x;
        marker.pose.orientation.y = normal_vec.y;
        marker.pose.orientation.z = normal_vec.z;
        marker.pose.orientation.w = 1.0;

        // Set marker scale and color
        marker.scale.x = 0.02;  // Arrow shaft diameter
        marker.scale.y = 0.03;  // Arrow head diameter
        marker.scale.z = 0.1;   // Arrow head length
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // Publish the marker
        uncommon_points_pub.publish(marker);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "centroid_normal_node");
    ROS_INFO("Node started");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub(nh, "/centroid", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub(nh, "/kinect/depth/points", 1);

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cloud1_sub, cloud2_sub);
    sync.registerCallback(boost::bind(&pointCloudCallback, _1, _2));
    
    uncommon_points_pub = nh.advertise<visualization_msgs::Marker>("/centroid_normal_marker", 1);

    ros::spin();

    return 0;
}