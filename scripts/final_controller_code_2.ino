#include <Wire.h> 
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>


// ====== ___IMPORTANT___ ====== //
const int totalMotors = 1 ; 
String jointNames[6]  = {"Base", "Shoulder Joint", "Elbow Joint", "Lower Wrist Joint", " Upper Wrist Joint", "Suction Joint" } ;
// ================ ====== //


// ----------- Motor Parameter Variables  ----------- //
int dirPin[totalMotors]  = {2} ; 
int stepPin[totalMotors] = {3} ;
int motorInterfaceType   = 1 ;
int maxSpeed             = 500 ;
int maxAccel             = 2000 ;

AccelStepper stepper[totalMotors] = {
  AccelStepper(motorInterfaceType, stepPin[0], dirPin[0])
//  ,AccelStepper(motorInterfaceType, stepPin[0], dirPin[0])
//  ,AccelStepper(motorInterfaceType, stepPin[0], dirPin[0])
//  ,AccelStepper(motorInterfaceType, stepPin[0], dirPin[0])
//  ,AccelStepper(motorInterfaceType, stepPin[0], dirPin[0])
//  ,AccelStepper(motorInterfaceType, stepPin[0], dirPin[0])
};


// ----------- Limit Switch/Homing Parameter Variables  ----------- //
int limitSwitch[totalMotors] = {13} ;
int homeDir[totalMotors]     = {1} ; // -1 -> Left ; 1 -> Right 
int homeAngle                = 10 ;
int errorCorrectionAngle     = 10 ;


// ----------- Magnetic Sensor Variables ----------- //
word highbyte; 
int quadrantNumber, previousquadrantNumber; 
int flag                 = 1 ; 
int rawAngle; 
int magnetStatus         = 0; 
int lowbyte; 
float degAngle; 
float numberofTurns      = 0; 
float correctedAngle     = 0;
float startAngle         = 0; 
float totalAngle         = 0; 
float previoustotalAngle = 0; 


// ----------- Error Connection Variables  ----------- //
int motorToDiscRotation          = {6840} ; // how many times a motor will rotate to rotate disc full 360 degree  
int discPerRotation[totalMotors] = {360} ;  // Encoder Value to rotate disc 1 degree 
int actualValue[totalMotors] ;           // -_ provide in degree 
int desireValue[totalMotors] = {100} ;     // -
int errorTolerance = 3 ; // encoder value 


// ----------- ROS Setup  ----------- //
ros::NodeHandle nh;
void subCallback(const std_msgs::Int32MultiArray& msg) { 
  int data_size = msg.data_length;
  for (int curntMotor = 0 ; curntMotor < data_size ; curntMotor++){
    desireValue[curntMotor] = msg.data[curntMotor] ; 
  }
}
ros::Subscriber<std_msgs::Int32MultiArray>
sub("joint_angle_array", &subCallback );


void setup() {

  Serial.begin(115200); 

  
  // ----------- Magnetic Sensor  Parameter Setup ----------- //
  Wire.begin();
  Wire.setClock(800000L);
  
  Serial.println("---- Encoder Configuration Starting ---- ") ;
  for (int curntMotor = 0 ; curntMotor < totalMotors ; curntMotor ++ ){
    Serial.print(jointNames[curntMotor]) ;
    Serial.print(" - Encoder : checkMagnentPresence") ;
    checkMagnetPresence();
    Serial.println(" - Done ") ;
  }

  
  // ----------- Motor Parameter Setup ----------- //
  for ( int curntMotor = 0 ; curntMotor < totalMotors ; curntMotor++){
    stepper[curntMotor].setMaxSpeed(maxSpeed);
    stepper[curntMotor].setAcceleration(maxAccel);
   }

  nh.initNode();
  nh.subscribe(sub);
  
  // ----------- Homing  ----------- // 
  
  // syncBot() ; 
  ReadRawAngle();
  startAngle = degAngle;

  
}


void loop() {

  ReadRawAngle(); 
  correctAngle(); 
  checkQuadrant(); 
  actualValue[0] = totalAngle ;
  
  int error = (actualValue[0]) - (desireValue[0]) ;
  Serial.println("--------------------------") ;
  Serial.print("Current Encoder Value : ") ;
  Serial.println(actualValue[0]) ;
  Serial.print("Desired Encoder Value :") ;
  Serial.println(desireValue[0]) ;
  Serial.print("Error : ") ;
  Serial.println(error) ;
  Serial.println("--------------------------") ;

  if ( abs(error) >= 3 ) {

  
  int dir =  error/abs(error) ;
  long targetPosition = stepper[0].currentPosition() + stepsForDegrees(errorCorrectionAngle )*dir ;
//   stepper[0].setDirection(e rror/abs(error)) ;
  stepper[0].moveTo(targetPosition);
  stepper[0].runToPosition();
  }

 nh.spinOnce();

}


void moveMotor(int curntMotor, int moveAngle) {

  long targetPosition = stepper[curntMotor].currentPosition() + stepsForDegrees(moveAngle);
  stepper[curntMotor].moveTo(targetPosition);
  stepper[curntMotor].runToPosition();
  
}


long stepsForDegrees(float degrees) {
  float stepsPerRevolution = 200.0;
  float degreesPerStep = 360.0 / stepsPerRevolution;
  return degrees / degreesPerStep;
}

void syncBot() {
  Serial.println("---- Starting Homing ---- ") ;  
  for (int curntMotor = 0 ; curntMotor < totalMotors ; curntMotor++){
    Serial.print(jointNames[curntMotor]) ;
    Serial.print(" - Motor : homing") ;
    while (!digitalRead(7)) moveMotor(0, homeAngle) ;
    Serial.println(" - Done ") ;
  }
}

void correctAngle()
{
  correctedAngle = degAngle - startAngle; 
  if(correctedAngle < 0) {
  correctedAngle = correctedAngle + 360; }
  else
  {
    //do nothing
  }
 }

void ReadRawAngle()
{ 
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625; 

  
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
  
}

void checkQuadrant()
{
  if(correctedAngle >= 0 && correctedAngle <=90)
  {
    quadrantNumber = 1;
  }
  if(correctedAngle > 90 && correctedAngle <=180)
  {
    quadrantNumber = 2;
  }
  if(correctedAngle > 180 && correctedAngle <=270)
  {
    quadrantNumber = 3;
  }
  if(correctedAngle > 270 && correctedAngle <360)
  {
    quadrantNumber = 4;
  }

  if(quadrantNumber != previousquadrantNumber) 
  {
    if(quadrantNumber == 1 && previousquadrantNumber == 4)
    {
      numberofTurns++; 
    }

    if(quadrantNumber == 4 && previousquadrantNumber == 1)
    {
      numberofTurns--; 
    }

    previousquadrantNumber = quadrantNumber;  
  
  }  
  totalAngle = (numberofTurns*360) + correctedAngle; }







void checkMagnetPresence()
{  
 
  while((magnetStatus & 32) != 32) {
    magnetStatus = 0;

    Wire.beginTransmission(0x36);
    Wire.write(0x0B); 
    Wire.endTransmission(); 
    Wire.requestFrom(0x36, 1); 

    while(Wire.available() == 0);  
    magnetStatus = Wire.read(); 

   }      
  

}
