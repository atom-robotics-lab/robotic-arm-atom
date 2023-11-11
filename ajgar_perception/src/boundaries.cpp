#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/boundary_estimation.h>

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pcl_extractor");

  // Get the .ply file address from the command line
  std::string file_path = argv[1];

  // Create a point cloud from the .ply file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPLYFile("/home/aakshar/catkin_ws/src/pcl_surface/output.ply", *cloud);

  // Estimate the boundary properties of the point cloud
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Boundary> boundary_estimator;
  boundary_estimator.setInputCloud(cloud);
  boundary_estimator.setRadiusSearch(0.05);
  boundary_estimator.compute();

  // Publish the boundary properties to the /boundaries topic
  ros::Publisher boundaries_publisher = ros::Publisher("boundaries", 
                                                    pcl::PointCloud<pcl::Boundary>::Ptr);
  boundaries_publisher.publish(boundary_estimator.getOutputCloud());

  // Spin until ROS is shutdown
  ros::spin();

  return 0;
}