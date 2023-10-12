#include <Wire.h> 
#include <AccelStepper.h>

const int totalMotors = 1 ;
int maxSpeed = 5000 ;
int maxAccel = 5000 ;

// ----------- Motor Parameter Intialization ----------- //
int dirPin[totalMotors] = {2} ;
int stepPin[totalMotors] = {3} ;
int motorInterfaceType = 1 ;

// AccelStepper stepper[totalMotors] ;

// ----------- Limit Switch Intialization ----------- //

int limitSwitch[totalMotors] = {13} ;

float curntJointState[totalMotors] = {0} ; 
float desiredJointState[totalMotors] = {0} ; 

// ----------- Homing Parameter Intialization ----------- //
int homeDir[totalMotors] = {1} ; // 0 -> Left ; 1 -> Right 

AccelStepper stepper_1 = AccelStepper(motorInterfaceType, stepPin[0], dirPin[0]);


long stepsForDegrees(float degrees) {
  float stepsPerRevolution = 800.0;
  float degreesPerStep = 360.0 / stepsPerRevolution;
  return degrees / degreesPerStep;
}


void setup() {

  Serial.begin(115200); 

  // for ( int curntMotor = 0 ; curntMotor > totalMotors ; curntMotor++){
  //   AccelStepper stepper[curntMotor] = {AccelStepper(motorInterfaceType, stepPin[curntMotor], dirPin[curntMotor])} ;
  //   stepper[curntMotor].setMaxSpeed(maxSpeed);
  //   stepper[curntMotor].setAcceleration(maxAccel);
  //   pinMode(limitSwitch[curntMotor], INPUT) ;
  //  }

  // iskeBCMC() ;

  }


void loop() {
  

  iskeBCMC() ;

}

// void syncBot() {

//   Serial.println("1") ;
//   for (int curntMotor = 0; curntMotor < totalMotors ; curntMotor++ ){
//   Serial.println("2") ;

//     while(!digitalRead(limitSwitch[curntMotor])){
//       if (homeDir[curntMotor]) { moveMotor(curntMotor, 200, 0) ; }
//     }
//     Serial.print("Done Homing : ") ;
//     Serial.println(curntMotor) ;
//   }
// }


// void moveMotor(int curntMotor, int moveAngle, int dir){
  
//   Serial.println(stepper[curntMotor].currentPosition()) ;
//   long targetPosition = stepper[curntMotor].currentPosition() + stepsForDegrees(200);
//   Serial.println("6") ;
//   Serial.print(curntMotor) ;
//   stepper[curntMotor].moveTo(targetPosition);
//   Serial.println("7") ;
//   stepper[curntMotor].runToPosition();
//   Serial.println("8") ;


// }

void iskeBCMC() {


  long targetPosition = stepper_1.currentPosition() + stepsForDegrees(200);
  stepper_1.moveTo(targetPosition);
  stepper_1.runToPosition();
  Serial.println("hhehehe") ;
}



