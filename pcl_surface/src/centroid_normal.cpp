#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/make_shared.hpp>

ros::Publisher uncommon_points_pub;

// Define cloud2 at a higher scope
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg, const sensor_msgs::PointCloud2ConstPtr& cloud2_msg)
{

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

    pcl::Normal centroid_normal;

    pcl::PointCloud<pcl::PointXYZ> uncommon_points;

    for (size_t i = 0; i < cloud2->points.size(); ++i) {
        float x_diff = cloud1->points[i].x - cloud2->points[i].x;
        float y_diff = cloud1->points[i].y - cloud2->points[i].y;
        if (fabs(x_diff) < 0.02 && fabs(y_diff) < 0.02) { // Check if the index is within bounds
            centroid_normal = normals->points[i]; // Access the normal corresponding to the current point in cloud2

            // Create a marker for the normal at the centroid
            visualization_msgs::Marker marker;
            marker.header.frame_id = "camera_depth_optical_frame";
            marker.header.stamp = ros::Time::now();
            marker.ns = "normals";
            marker.id = i;  // Use a unique ID for each marker (e.g., based on the index)
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            // Set the position of the marker at the centroid point from cloud1
            geometry_msgs::Point point;
            point.x = cloud1->points[i].x; // Use the index 'i' to access the corresponding point in cloud1
            point.y = cloud1->points[i].y;
            point.z = cloud1->points[i].z;
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
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "centroid_normal_node");
    ROS_INFO("Node started");

    ros::NodeHandle nh;

    // Subscribe to the /centroid and /mask topics using message_filters
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub(nh, "/centroid", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub(nh, "/mask", 1);

    // Define the synchronization policy and synchronizer
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cloud1_sub, cloud2_sub);
    sync.registerCallback(boost::bind(&pointCloudCallback, _1, _2));

    // Create a publisher for visualization markers
    uncommon_points_pub = nh.advertise<visualization_msgs::Marker>("/centroid_normal_marker", 1);

    ros::spin();

    return 0;
}
