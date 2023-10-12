#include <Wire.h> 
#include <AccelStepper.h>

int totalMotors = 1 ;

// ----------- Motor Parameter Intialization ----------- //
int dirPin[totalMotors] = {2} ;
int stepPin[totalMotors] = {3} ;
int motorInterfaceType = 1

AccelStepper stepper[totalMotors] = {AccelStepper(motorInterfaceType, stepPin[0], dirPin[0])} ;


// ----------- Encoder Parameter Intialization ----------- //
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


// ----------- Controller Parameter Intialization ----------- //

int desiredAngle[totalMotors] = {60} ;
int actualAngle[totalMotors] = {0} ;
int curntEncoderValue[totalMotors] = {0} ;
int exptEncoderValue[totalMotors] = {0} ; 
int unitEncoderAngleValue = ;
int unitAngleEncoderValue = ;
int angleToRotate = 0 ;


// ----------- Setup function ----------- //
void setup()
{

  Serial.begin(115200); 

// Motor
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(5000);

// Encoder 
  Wire.begin(); 
	Wire.setClock(800000L); 
  Serial.println("checkMagnentPresence") ;
  checkMagnetPresence();  
  startAngle = degAngle; 



 
}

void loop()
{    


  for (int curntMotor = 0 ; curntMotor > totalMotors ; curntMotor++) {
    
    ReadRawAngle(); 
    correctAngle(); 
    checkQuadrant();
    curntEncoderValue[curntMotors] = totalAngle ;
    Serial.print("Motor %d Encoder reading : ") ; 
    Serial.println(curntEncoderValue[totalMotors]);

    exptEncoderValue[curntMotors] =  unitEncoderAngleValue * desiredAngle[totalMotors] ; 
    
    angleToRotate = exptEncoderValue - curntEncodrValue ; 

    long targetPosition = stepper.currentPosition() + stepsForDegrees(); 
    stepper.moveTo(targetPosition);
    stepper.runToPosition();
    Serial.println(stepper.currentPosition());


    delay(2000) ;}
    
}

void ReadRawAngle()
{ 
  
  Wire.beginTransmission(0x36); 
  Wire.write(0x0D); 
  Wire.endTransmission(); 
  Wire.requestFrom(0x36, 1); 
  
  while(Wire.available() == 0); 
  lowbyte = Wire.read(); 
 
 
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); 
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
   highbyte = highbyte << 8; 
  rawAngle = highbyte | lowbyte; 

  degAngle = rawAngle * 0.087890625; 

  
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

  //Quadrant 4
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
  
 totalAngle = (numberofTurns*360) + correctedAngle;   }

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

long stepsForDegrees(float degrees) {

  float stepsPerRevolution = 800.0;
  float degreesPerStep = 360.0 / stepsPerRevolution;
  return degrees / degreesPerStep;
}    
  
 
}

