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




    // Calculate the average x and y coordinates
    float avg_x = 0.0;
    float avg_y = 0.0;

    // Only consider the first and last points in the point cloud
    if (cloud1->size() >= 2) {
        avg_x = (cloud1->front().x + cloud1->back().x) / 2.0;
        avg_y = (cloud1->front().y + cloud1->back().y) / 2.0;
        }
        PointType centroid_point;        // PointXYZ typedef for mean of all points
        PointType centroid_up_point;
        PointType centroid_down_point;
        PointType centroid_right_point;
        PointType centroid_left_point;

        int centroid_index = 0;       // for index of point with similar coordinates
        bool found_centroid_up_point = false;
        bool found_centroid_down_point = false;
        bool found_centroid_right_point = false;
        bool found_centroid_left_point = false;

        // Search for a point with x and y values close to the averages
        for (const pcl::PointXYZ& point : cloud1->points) {


            // condition for centroid point
            if ( std::abs(point.x - avg_x) < 0.001 && std::abs(point.y - avg_y) < 0.001 ) {
                centroid_point = point;
                centroid_index+=1;

                for (const pcl::PointXYZ& point : cloud1->points) {

                    if ( found_centroid_up_point == false && 
                            abs(point.x - centroid_point.x) <= 0.0002 && 
                            (point.y - centroid_point.y) > 0.00001 ) {
                        centroid_up_point = point;
                        found_centroid_up_point = true;
                        }


                    else if ( found_centroid_down_point == false && 
                            abs(point.x - centroid_point.x) <= 0.0002 && 
                            (point.y - centroid_point.y) < 0.00001 ) {
                        centroid_down_point = point;
                        found_centroid_down_point = true;
                        }


                    else if ( found_centroid_right_point == false && 
                            abs(point.y - centroid_point.y) <= 0.0002 && 
                            (point.x - centroid_point.x) < 0.00001 ) {
                        centroid_right_point = point;
                        found_centroid_right_point = true;
                        }


                    else if ( found_centroid_left_point == false && 
                            abs(point.y - centroid_point.y) <= 0.0002 && 
                            (point.x - centroid_point.x) > 0.00001 ) {
                        centroid_left_point = point;
                        found_centroid_left_point = true;
                        }
                    }    
                break;
            }
        }

        

        pcl::PointCloud<pcl::PointXYZ> centroid_cloud;        // Create a new point cloud and push the centroid point into it
        centroid_cloud.push_back(centroid_point);        // Pushing back the Centroid Point into centroid_cloud
        centroid_cloud.push_back(centroid_up_point);
        centroid_cloud.push_back(centroid_down_point);
        centroid_cloud.push_back(centroid_right_point);
        centroid_cloud.push_back(centroid_left_point);



    // NormalEstimation analyzes local geometry around each point to infer the orientation of surfaces.
    // The function considers neighboring points within a certain range to ensure 
    // that the computed normals capture the local shape of the point cloud.

    pcl::NormalEstimation<PointType, NormalType> normal_estimation;
    normal_estimation.setInputCloud(cloud1);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(cloud1);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(0.3);     // Set the radius for finding neighbors
    normal_estimation.compute(*normals);    // Compute the normals




    // Access the normal at the center point
    Eigen::Vector3f center_normal(
        normals->points[centroid_index].normal_x,
        normals->points[centroid_index].normal_y,
        normals->points[centroid_index].normal_z
    );
    center_normal.normalize();    // Normalize the normal vector converts a vector into unit vector

    // std::cout << "Normal of the point at the center: " << center_normal.transpose() << std::endl;




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




    //Publishing Centroid(PointCloud2) and Normal(Vector3)
    centroid_pub.publish(centroid_msg);
    normals_pub.publish(normal_msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "normals_node"); ros::NodeHandle nh;
    normals_pub = nh.advertise<geometry_msgs::Vector3>("/normals", 1);
    centroid_pub = nh.advertise<sensor_msgs::PointCloud2>("/centroid_cloud", 1);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/mask", 1, pointCloudCallback);
    ros::spin();
    return 0;
}