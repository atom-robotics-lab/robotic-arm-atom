#include <Wire.h> 
#include <AccelStepper.h>

const int totalMotors = 1 ;

// ----------- Motor Parameter Variables  ----------- //
int dirPin[totalMotors] = {2} ;
int stepPin[totalMotors] = {3} ;
int motorInterfaceType = 1 ;
int maxSpeed = 5000 ;
int maxAccel = 5000 ;
AccelStepper stepper[totalMotors] = {
  AccelStepper(motorInterfaceType, stepPin[0], dirPin[0])
};

// ----------- Limit Switch Variables  ----------- //
int limitSwitch[totalMotors] = {13} ;
float curntJointState[totalMotors] = {0} ; 
float desiredJointState[totalMotors] = {0} ; 

// ----------- Homing Parameter Variables  ----------- //
int homeDir[totalMotors] = {1} ; // 0 -> Left ; 1 -> Right 
int homeAngle = 200 ;
int errorCorrectionAngle = 100 ;

// ----------- Magnetic Sensor Variables ----------- //
int magnetStatus = 0; 
int lowbyte; 
word highbyte; 
int rawAngle; 
float degAngle; 
int quadrantNumber, previousquadrantNumber; 
float numberofTurns = 0; 
float correctedAngle = 0;
float startAngle = 0; 
float totalAngle = 0; 
float previoustotalAngle = 0; 


// ----------- Error Connection Variables  ----------- //
int actualValue[totalMotors] = {0} ;
int desireValue[totalMotors] = {1290} ;


void setup() {

  Serial.begin(115200); 
  pinMode(8, INPUT) ;
  pinMode(13, OUTPUT) ;
  digitalWrite(13, HIGH) ;


  // ----------- Magnetic Sensor  Parameter Setup ----------- //
  Wire.begin();
  Wire.setClock(800000L);
  Serial.println("checkMagnentPresence") ;
  checkMagnetPresence();
  startAngle = degAngle;

  
  // ----------- Motor Parameter Setup ----------- //
  for ( int curntMotor = 0 ; curntMotor < totalMotors ; curntMotor++){
    stepper[curntMotor].setMaxSpeed(maxSpeed);
    stepper[curntMotor].setAcceleration(maxAccel);
   }

  // ----------- Homing  ----------- // 
  // syncBot() ; 
}


void loop() {

  ReadRawAngle(); 
  correctAngle(); 
  checkQuadrant(); 
  actualValue[0] = totalAngle ; 
  int error = actualValue[0] - desireValue[0] ;
  Serial.print("Error : ") ;
  Serial.println(error) ;


  if ( abs(error) >= 5 ) {
  
  int dir =  error/abs(error) ;
  long targetPosition = stepper[0].currentPosition() + stepsForDegrees(errorCorrectionAngle);
  // stepper[0].setDirection(error/abs(error)) ;
  stepper[0].moveTo(targetPosition);
  stepper[0].runToPosition();
  }


}





void moveMotor(int curntMotor, int moveAngle) {

  long targetPosition = stepper[curntMotor].currentPosition() + stepsForDegrees(moveAngle);
  stepper[curntMotor].moveTo(targetPosition);
  stepper[curntMotor].runToPosition();
  
}


long stepsForDegrees(float degrees) {
  float stepsPerRevolution = 800.0;
  float degreesPerStep = 360.0 / stepsPerRevolution;
  return degrees / degreesPerStep;
}



void syncBot() {

  Serial.println("x-----------------------x") ;
  Serial.println("Start Homing : ") ;


  for (int curntMotor = 0 ; curntMotor < totalMotors ; curntMotor++){
    
    Serial.print("|___ Homing Motor : ") ; 
    Serial.println(curntMotor) ;
    
    while (!digitalRead(8)) {
      
      moveMotor(0, homeAngle) ;
    }

    Serial.print("--|___ done Homing ") ; 

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



