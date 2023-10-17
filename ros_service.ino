#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include<std_msgs/String.h>
#include <ajgar/bool_service.h>

ros::NodeHandle nh;
std_msgs::String str_msg;
using ajgar::bool_service;

void subCallback(const std_msgs::Int32MultiArray& msg) {
  nh.loginfo("sub callback");
  // Process the received array
  int data_size = msg.data_length;
  for (int i = 0; i < data_size; i++) {
    if (msg.data[i] % 2 == 0) {
      nh.loginfo("led on");
      digitalWrite(13, HIGH);
      delay(1000);
    }
    else {
      nh.loginfo("led off");
      digitalWrite(13, LOW);
      delay(1000);
    }
  }
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("joint_angle_array", &subCallback );

void boolCallback(bool_service::Request& req, bool_service::Response& res) {
  int i;
  if ((i++) % 2)
    res.output = true;
  else
    res.output = false;
}

ros::ServiceServer<bool_service::Request, bool_service::Response> server("bool_service", boolCallback);

ros::Publisher pub("joint_angle_feedback", &str_msg);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertiseService(server);
  nh.advertise(pub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
