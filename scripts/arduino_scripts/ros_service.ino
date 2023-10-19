#include <ros.h>
#include <std_msgs/String.h>
#include <ajgar/bool_service.h>

ros::NodeHandle nh;
std_msgs::String str_msg, msg;
using ajgar::bool_service;

int angle_array[6];
int j = 0;

void subCallback(const std_msgs::String& msg) {
  nh.loginfo("sub callback");
  // Process the received array
  nh.loginfo(msg.data);
  String data = msg.data ; 
  int msg_data = data.length() ;
  for (int i = 0; i < msg_data; i++) {
    String ran = String(data[i]);
    char my_string[50]  ;
    ran.toCharArray(my_string, 50) ; 
    if (msg.data[i] == ',') {
      nh.loginfo("pass");
      delay(50);
    }
    else {
      angle_array[j] = my_string;
      nh.loginfo(angle_array[j]);
      delay(50);
    }
    j++;
  }
}

ros::Subscriber<std_msgs::String> sub("joint_angle_array", &subCallback );

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
  str_msg.data = "ros service publisher";
  pub.publish(&str_msg);
  nh.spinOnce();
  delay(1);
}