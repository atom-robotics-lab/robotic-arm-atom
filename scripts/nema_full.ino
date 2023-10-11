#include <Wire.h> //This is for i2C

#include <AccelStepper.h>

int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber, previousquadrantNumber; //quadrant IDs
float numberofTurns = 0; //number of turns
float correctedAngle = 0; //tared angle - based on the startup value
float startAngle = 0; //starting angle
float totalAngle = 0; //total absolute angular displacement
float previoustotalAngle = 0; //for the display printing

float degree_to_encoder = 1/19;
float encoder_angle = 0;
float error = 0;

#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  
  Serial.begin(9600);
  
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(350);
  
  Wire.begin(); //start i2C  
  Wire.setClock(800000L); //fast clock
  
  Serial.println("checkMagnentPresence") ;
  checkMagnetPresence(); //check the magnet (blocks until magnet is found)
  Serial.println("checkMagnentPresence") ;
  ReadRawAngle(); //make a reading so the degAngle gets updated
  startAngle = degAngle; //update startAngle with degAngle - for taring
}

void loop() {
  
  ReadRawAngle();
  correctAngle(); //tare the value
  checkQuadrant(); //check quadrant, check rotations, check absolute angular position
  Serial.println(totalAngle);
  
  Serial.println("Enter the desired angle (in degrees) and press Enter:");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(10);
  }

  // Read user input until Enter key is pressed
  String input = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      break;  // Exit loop when Enter key is pressed
    }
    input += c;
  }

  // Convert the user input to a float (desired angle in degrees)
  float desiredAngle = input.toFloat()/4;

  // Calculate the target position in steps
  long targetPosition = stepsForDegrees(desiredAngle);

  // Set the target position:
  stepper.moveTo(targetPosition);
  stepper.runToPosition();

  // Print the current position (in steps) and angle (in degrees)
  Serial.print("Current Position (steps): ");
  Serial.println(stepper.currentPosition()*4);
  Serial.print("Current Angle (degrees): ");
  Serial.println(currentAngle()*4);

  ReadRawAngle();
  correctAngle(); //tare the value
  checkQuadrant(); //check quadrant, check rotations, check absolute angular position
  Serial.println(totalAngle);
  ErrorCorrect(desiredAngle);  
  
  delay(100);
}

void ErrorCorrect(float desiredAngle) 
{
  float degree_to_encoder = 1/19;
  Serial.println(desiredAngle);
  encoder_angle = 2*desiredAngle*degree_to_encoder;
  error = encoder_angle - totalAngle;
  if (abs(error) <= 1) {
    Serial.println("No Error");
  }
  else {
    while (abs(error) >=1) {
      long targetPosition = stepsForDegrees(-error/degree_to_encoder);
      // Set the target position:
      stepper.moveTo(targetPosition);
      stepper.runToPosition();
      encoder_angle = desiredAngle*degree_to_encoder;
      error = encoder_angle - totalAngle;
      Serial.print("Error is: ");
      Serial.println(error);
      
      return error;
    }
  }

  } 

// Function to calculate the steps needed for a given number of degrees:
long stepsForDegrees(float degrees) {
  float stepsPerRevolution = 800.0;
  float degreesPerStep = 360.0 / stepsPerRevolution;
  return degrees / degreesPerStep;
}

// Function to calculate the current angle (in degrees):
float currentAngle() {
  float stepsPerRevolution = 800.0;
  float degreesPerStep = 360.0 / stepsPerRevolution;
  return stepper.currentPosition() * degreesPerStep;
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

void correctAngle()
{
  //recalculate angle
  correctedAngle = degAngle - startAngle; //this tares the position

  if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
  {
  correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
  //Serial.print("Corrected angle: ");
  //Serial.println(correctedAngle, 2); //print the corrected/tared angle  
}

void checkQuadrant()
{
  /*
  //Quadrants:
  4  |  1
  ---|---
  3  |  2
  */

  //Quadrant 1
  if(correctedAngle >= 0 && correctedAngle <=90)
  {
    quadrantNumber = 1;
  }

  //Quadrant 2
  if(correctedAngle > 90 && correctedAngle <=180)
  {
    quadrantNumber = 2;
  }

  //Quadrant 3
  if(correctedAngle > 180 && correctedAngle <=270)
  {
    quadrantNumber = 3;
  }

  //Quadrant 4
  if(correctedAngle > 270 && correctedAngle <360)
  {
    quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if(quadrantNumber != previousquadrantNumber) //if we changed quadrant
  {
    if(quadrantNumber == 1 && previousquadrantNumber == 4)
    {
      numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if(quadrantNumber == 4 && previousquadrantNumber == 1)
    {
      numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber = quadrantNumber;  //update to the current quadrant

  }  

  totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
}

void checkMagnetPresence()
{  

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

}
