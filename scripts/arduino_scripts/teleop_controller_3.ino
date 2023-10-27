#include <Wire.h>
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/String.h>


/* ----------- Motor Parameter Variables  ----------- */
int       motorInterfaceType       = 1 ;
int       motordir                 = 1 ; // 1 : cw , -1 : acw
int       operationMode            = 1 ; // default operationMode -- Teleop 
const int totalMotors              = 1 ;
const int dirPin[totalMotors]      = {2} ;
const int stepPin[totalMotors]     = {3} ;
const int teleopSpeed[totalMotors] = {1000} ;
const int maxSpeed                 = 500 ;
const int maxAccel                 = 2000;
String    jointNames[6]            = {"Base", 
                                      "Shoulder Joint",
                                      "Elbow Joint", 
                                      "Lower Wrist Joint", 
                                      "Upper Wrist Joint", 
                                      "Suction Joint"};

// TODO : Put it into for loop 
AccelStepper stepper[totalMotors] = {
  AccelStepper(motorInterfaceType, stepPin[0], dirPin[0])
  //  ,AccelStepper(motorInterfaceType, stepPin[1], dirPin[1])
  //  ,AccelStepper(motorInterfaceType, stepPin[2], dirPin[2])
  //  ,AccelStepper(motorInterfaceType, stepPin[3], dirPin[3])
  //  ,AccelStepper(motorInterfaceType, stepPin[4], dirPin[4])
  //  ,AccelStepper(motorInterfaceType, stepPin[5], dirPin[5])
};



/* ----------- Functions  ----------- */
void rosserialPrint(String data) ;



/* ----------- Homing Parameter Variables  ----------- */
int limitSwitch[totalMotors] = {13};    
int homeDir[totalMotors]     = {1};  // -1 -> Left ; 1 -> Right
int errorCorrectionAngle     = 10;
int homeAngle                = 0 ;



/* ----------- Magnetic Sensor Variables ----------- */
word highbyte;
int quadrantNumber, previousquadrantNumber;
int flag                 = 1;
int rawAngle;
int magnetStatus         = 0;
int lowbyte;
float degAngle;
float numberofTurns      = 0;
float correctedAngle     = 0;
float startAngle         = 0;
float totalAngle         = 0;
float previoustotalAngle = 0;



/* ----------- Error Connection Variables  ----------- */
// motorToDiscRotation : Numbers of time motor will rotate in order
//                       to rotate main disc 360 degree
int motorToDiscRotation          = {6840};        

// discPerRotation : Number of points Encoder reads while main disc 
//                   rotate 360 degree
int discPerRotation[totalMotors] = {360}; 
int actualValue[totalMotors]     = {0};     // Current motor reading  
int desireValue[totalMotors]     = {100};   // Desire motor reading 
int errorTolerance               = 3;       // Acceptable Encoder reading range              



/* ----------- ROSserial Setup  ----------- */
char nodeToSubs[50] = "roboticARM/PC_Arduino/operationString" ;

ros::NodeHandle nh;
std_msgs::String msg;

/*
#---------------------------------------------------------------#
# Function Name : subCallback
# Arguments     : msg
# Description   : whenever subscriber publish data , this function 
                  is called containing data regarding operationMode
                  along with angle to rotate and direction.
                  This function extract data and assign it to respective
                  motors 
#---------------------------------------------------------------#
*/
void subCallback(const std_msgs::String &msg){

  String data       = msg.data;            // ___ to calculate length string received 
  int    msg_data   = data.length();       // _| 
  int    curntMotor = 0 ;                  
  int    msgVal     = 0 ;
  int    unitPlace  = 10 ;
  operationMode     = int(msg.data[0]) - int('0') ;
  
  for (int msg_length = 2; msg_length <= msg_data; msg_length++){
    
    if ( msg.data[msg_length] == ',' || msg_length == msg_data ) { 
          desireValue[curntMotor] = msgVal ; 
          msgVal = 0  ; 
          curntMotor++ ; }
    
    else { msgVal = msgVal*unitPlace + (int(msg.data[msg_length]) - int('0')) ; } 
    
    }}

ros::Subscriber<std_msgs::String> sub(nodeToSubs, &subCallback );


const char infoString_0[] PROGMEM = "ATOM ROBOTICS LAB";
const char infoString_1[] PROGMEM = "AJGAR ARM";
const char infoString_2[] PROGMEM = "ENCODER CALIBRATION STARTING ..";
const char infoString_3[] PROGMEM = " ";
const char infoString_4[] PROGMEM = "   \\__ checking ..";
const char infoString_5[] PROGMEM = "    \\__ calibration done";
const char infoString_6[] PROGMEM = "ENCODER CALIBRATION DONE !";


const char* const infoString_ptr[] PROGMEM = {infoString_0, infoString_1, 
                                              infoString_2, infoString_3,
                                              infoString_4, infoString_5,
                                              infoString_6};


void setup(){


  nh.initNode();
  nh.subscribe(sub);

  char infoStringBuffer[70] ;
  int  requiredString = 3 ;

  for (int curntPtr = 0 ; curntPtr < requiredString ; curntPtr++){
  strcpy_P(infoStringBuffer, (char*)pgm_read_word(&(infoString_ptr[curntPtr])));
  nh.loginfo (infoStringBuffer) ;
  }
  


  /* ----------- Magnetic Sensor  Parameter Setup ----------- */
  Wire.begin();
  Wire.setClock(800000L);


  for (int curntMotor = 0; curntMotor < 5; curntMotor++)
  {
    
     nh.loginfo(jointNames[curntMotor].c_str()) ;
     
     
     strcpy_P(infoStringBuffer, (char*)pgm_read_word(&(infoString_ptr[4])));
     nh.loginfo (infoStringBuffer) ;
  //   checkMagnetPresence();

     strcpy_P(infoStringBuffer, (char*)pgm_read_word(&(infoString_ptr[5])));
     nh.loginfo (infoStringBuffer) ;

     strcpy_P(infoStringBuffer, (char*)pgm_read_word(&(infoString_ptr[3])));
     nh.loginfo (infoStringBuffer) ;
     
  }
  
 strcpy_P(infoStringBuffer, (char*)pgm_read_word(&(infoString_ptr[6])));
 nh.loginfo (infoStringBuffer) ;
    
  
  // ----------- Motor Parameter Setup ----------- //
  for (int curntMotor = 0; curntMotor < totalMotors; curntMotor++)
  {
    stepper[curntMotor].setMaxSpeed(maxSpeed);
    stepper[curntMotor].setAcceleration(maxAccel);
  }

  // ----------- Homing  ----------- //
  // syncBot() ;
  //  ReadRawAngle();
  //  startAngle = degAngle;
}



void loop(){

  // Teleop Mode
  if (operationMode == 0) {
    
    for (int curntMotor = 0 ; curntMotor < totalMotors; curntMotor++) {
      
      if      (desireValue[curntMotor] == 997 ) { stepper[curntMotor].setSpeed(-1*teleopSpeed[curntMotor]);  }
      else if (desireValue[curntMotor] == 998 ) { stepper[curntMotor].setSpeed(0);                           }
      else if (desireValue[curntMotor] == 999 ) { stepper[curntMotor].setSpeed(1*teleopSpeed[curntMotor]);   }

      stepper[curntMotor].runSpeed();
      nh.spinOnce();
  }
  
  nh.spinOnce();
}
nh.spinOnce();
}



/*
#---------------------------------------------------------------#
# Function Name : moveMotor
# Arguments     : curntMotor , moveAngle
# Description   : take motor which needs to be moved in accordance with given angle
#---------------------------------------------------------------#
*/
void moveMotor(int curntMotor, int moveAngle){

  long targetPosition = stepper[curntMotor].currentPosition() + stepsForDegrees(moveAngle);
  stepper[curntMotor].moveTo(targetPosition);
  stepper[curntMotor].runToPosition();
  }



/*
#---------------------------------------------------------------#
# Function Name : stepsForDegrees
# Arguments     : degrees
# Description   : convert degrees into step needed for stepper to move
#---------------------------------------------------------------#
*/
long stepsForDegrees(float degrees){
  
  float stepsPerRevolution = 200.0;
  float degreesPerStep = 360.0 / stepsPerRevolution;
  return degrees / degreesPerStep;
}



/*
#---------------------------------------------------------------#
# Function Name : syncBot
# Arguments     : none
# Description   : whenever call sync the bot wrt homing position , 
                  and reset encoder values to zero 
#---------------------------------------------------------------#
*/
void syncBot(){
    
  Serial.println("---- Starting Homing ---- ");
  for (int curntMotor = 0; curntMotor < totalMotors; curntMotor++)
  {
    Serial.print(jointNames[curntMotor]);
    Serial.print(" - Motor : homing");
    while (!digitalRead(7))
      moveMotor(0, homeAngle);
    Serial.println(" - Done ");
  }
}



/*
#---------------------------------------------------------------#
# Function Name : correctAngle, ReadRawAngle, 
                  checkQuadrant, checkMagnentPresence
# Arguments     : none
# Description   : use in callibrating encoders and keep track of 
                  encoder values  
#---------------------------------------------------------------#
*/
void correctAngle() {

  correctedAngle = degAngle - startAngle;
  if (correctedAngle < 0) { correctedAngle = correctedAngle + 360; }
  else { } 
}



void ReadRawAngle(){
   
  Wire.beginTransmission(0x36); 
  Wire.write(0x0D);             
  Wire.endTransmission();       
  Wire.requestFrom(0x36, 1);    
  while (Wire.available() == 0) ; 
  lowbyte = Wire.read(); 
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); 
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  while (Wire.available() == 0) ;
  highbyte = Wire.read();
  highbyte = highbyte << 8; 
  rawAngle = highbyte | lowbyte; 
  degAngle = rawAngle * 0.087890625;
}



void checkQuadrant(){
  
  if (correctedAngle >= 0 && correctedAngle <= 90)  { quadrantNumber = 1; }
  if (correctedAngle > 90 && correctedAngle <= 180) { quadrantNumber = 2; }
  if (correctedAngle > 180 && correctedAngle <= 270){ quadrantNumber = 3; }
  if (correctedAngle > 270 && correctedAngle < 360) { quadrantNumber = 4; }
  if (quadrantNumber != previousquadrantNumber) {
    
    if (quadrantNumber == 1 && previousquadrantNumber == 4) { numberofTurns++; }
    if (quadrantNumber == 4 && previousquadrantNumber == 1) { numberofTurns--; }
    previousquadrantNumber = quadrantNumber;
  }
  
  totalAngle = (numberofTurns * 360) + correctedAngle;
}



void checkMagnetPresence() {

  while ((magnetStatus & 32) != 32)  {
    
    magnetStatus = 0;
    Wire.beginTransmission(0x36);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);
    while (Wire.available() == 0)  ;
    magnetStatus = Wire.read();
  }}



// REFERENCES AND RESOURCES 

/* __ ROSserial String FRAME Structure __ 
 * 
 * 0 : Run Using Telop 
 * 1 : Run Using Py Script
 * 
 * expected msg type to recieved : operation code-xx,xx,xx,xx,xx,xx  
 * 
 * OPERATION CODE : To decide teleop operation or py script operation  
 * xx : Desire motor value to rotate (in case of PY script) /    
 *      997,998,999 value to rotate motor cw/acw or to halt joints individually
 * 
 * eg : 0-0,1,2,1,2,0
 *      1-43,54,21,0,12,34
 * 
 *
 *
 * __ Encoder Code reference __
 * 
 * https://curiousscientist.tech/blog/as5600-magnetic-position-
 * 
 * __ Print Msg on Ros Serial __
 * 
 * void rosserialPrint() converts String into chars so to make it compatible with
 * loginfo function .
 * void rosserialPrint() function only accepts Strings , so convert char/int/floats
 * into string before . 
 * 
 * __ros.loginfo memory expensive task__
 * 
 * if have limited memory try not to use loginfo as it consumes lots of memory
 * 
 */  
