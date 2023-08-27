#include <ros/ros.h>
#include <pcl/features/boundary.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/surface/poisson.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/extract_indices.h>
#include <boost/make_shared.hpp>




ros::Publisher normals_pub;
ros::Publisher marker_pub;
ros::Publisher boundary_points_pub;
ros::Publisher cluster_points_pub;




void pointCloudCallback(const pcl::PCLPointCloud2ConstPtr& input_cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*input_cloud_msg, *pcl_cloud);

    // VoxelGrid Downsampling
    pcl::VoxelGrid<pcl::PointXYZ> down_sample;
    down_sample.setInputCloud(pcl_cloud);
    down_sample.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    down_sample.filter(*pcl_cloud_filtered);


    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pcl_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    ne.setRadiusSearch(0.03);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr pcl_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*pcl_cloud, *normals, *pcl_normals);

    sensor_msgs::PointCloud2 normals_msg;
    pcl::toROSMsg(*pcl_normals, normals_msg);
    normals_msg.header = pcl_conversions::fromPCL(input_cloud_msg->header);
    normals_pub.publish(normals_msg);

    // boundaries 
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation;
    boundary_estimation.setInputCloud(pcl_cloud);
    boundary_estimation.setInputNormals(normals);
    boundary_estimation.setSearchMethod(tree);
    boundary_estimation.setKSearch(15);  // You can adjust this value as needed

    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
    boundary_estimation.compute(*boundaries);


    // Clustering
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ec.setClusterTolerance(0.03);    // Adjust as needed
    // ec.setMinClusterSize(100000);       // Adjust as needed
    // ec.setMaxClusterSize(250000);     // Adjust as needed
    // ec.setInputCloud(pcl_cloud_filtered);
    // ec.extract(cluster_indices);

    // Find the largest cluster
    // int largest_cluster_index = 0;
    // size_t largest_cluster_size = 0;
    // for (size_t i = 0; i < cluster_indices.size(); ++i) {
    //     if (cluster_indices[i].indices.size() > largest_cluster_size) {
    //         largest_cluster_size = cluster_indices[i].indices.size();
    //         largest_cluster_index = i;
    //     }
    // }

    // Extract points of the largest cluster using pcl::PointIndices
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(pcl_cloud_filtered);
    // extract.setIndices(boost::make_shared<pcl::PointIndices>(cluster_indices[largest_cluster_index]));

    // Perform the extraction
    // extract.filter(*cluster_points);

    // sensor_msgs::PointCloud2 cluster_points_msg;
    // pcl::toROSMsg(*cluster_points, cluster_points_msg);
    // cluster_points_msg.header = pcl_conversions::fromPCL(input_cloud_msg->header);
    // cluster_points_pub.publish(cluster_points_msg);






    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_points_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < pcl_normals->points.size(); ++i) {
        if (boundaries->points[i].boundary_point) {
            pcl::PointXYZ boundary_point;
            boundary_point.x = pcl_normals->points[i].x;
            boundary_point.y = pcl_normals->points[i].y;
            boundary_point.z = pcl_normals->points[i].z;
            boundary_points_cloud->points.push_back(boundary_point);
        }
    }

    sensor_msgs::PointCloud2 boundary_points_msg;
    pcl::toROSMsg(*boundary_points_cloud, boundary_points_msg);
    boundary_points_msg.header = pcl_conversions::fromPCL(input_cloud_msg->header);
    boundary_points_pub.publish(boundary_points_msg);

    visualization_msgs::Marker arrow_marker;
    arrow_marker.header = pcl_conversions::fromPCL(input_cloud_msg->header);
    arrow_marker.ns = "normals";
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;

    for (size_t i = 0; i < pcl_normals->points.size(); ++i) {
        arrow_marker.id = static_cast<int>(i);
        arrow_marker.points.resize(2);
        arrow_marker.points[0].x = pcl_normals->points[i].x;
        arrow_marker.points[0].y = pcl_normals->points[i].y;
        arrow_marker.points[0].z = pcl_normals->points[i].z;
        arrow_marker.points[1].x = pcl_normals->points[i].x + pcl_normals->points[i].normal_x * 0.1;
        arrow_marker.points[1].y = pcl_normals->points[i].y + pcl_normals->points[i].normal_y * 0.1;
        arrow_marker.points[1].z = pcl_normals->points[i].z + pcl_normals->points[i].normal_z * 0.1;

        arrow_marker.scale.x = 0.005;
        arrow_marker.scale.y = 0.01;
        arrow_marker.scale.z = 0.0;
        if (boundaries->points[i].boundary_point) {
        // Set marker color for boundary points
        arrow_marker.color.r = 0.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 1.0;
        arrow_marker.color.a = 1.0;  // Blue color for boundaries
        marker_pub.publish(arrow_marker);
        ROS_INFO("Publishing Boundary Normal");

        }   
        

    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "normal_estimation_node");
    ros::NodeHandle nh;

    normals_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_normals_topic", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("normals_marker_topic", 1);
    boundary_points_pub = nh.advertise<sensor_msgs::PointCloud2>("boundary_points_topic", 1);
    cluster_points_pub = nh.advertise<sensor_msgs::PointCloud2>("cluster_points_topic", 1);


    ros::Subscriber sub = nh.subscribe<pcl::PCLPointCloud2>(
        "/kinect/depth/points", 1, pointCloudCallback);

    ros::spin();
    return 0;
}