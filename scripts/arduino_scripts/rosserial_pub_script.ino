#include <ros.h>
#include <std_msgs/String.h>

std_msgs::String str_msg;
ros::Publisher string_array_pub("arduino_controller", &str_msg);
ros::NodeHandle nh;


int potPin = A0 ; 
int turn = 0 ;
float accum = 0 ;

void setup() {
  nh.initNode();
  nh.advertise(string_array_pub);
}

void loop() {

  float potVal = analogRead(potPin) ;
  
  accum = potVal + accum ;
  turn = turn + 1 ;

  if (turn == 1000 ) {
    potVal = accum / 1000 ;
    turn = 0 ;
    accum = 0 ;
  }

  float potval = (potVal - 0) * (2.87 + 2.87) / (1023 - 0) -2.87 ;
  String potValStr = String(potval);
  const char* potValCStr = potValStr.c_str();

  str_msg.data = potValCStr ;
  string_array_pub.publish(&str_msg) ;
  nh.spinOnce() ;
  delay(10) ;


}
