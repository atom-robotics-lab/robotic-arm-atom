#include "ros/ros.h"
#include "ajgar_perception/AddTwoInts.h"
#include <iostream>

bool add(ajgar_perception::AddTwoInts::Request  &req, ajgar_perception::AddTwoInts::Response &res)
{
  //std::cout << req.inputPt << std::endl;
  res.outputPt = 1;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}

