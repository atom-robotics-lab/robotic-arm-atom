#include <AccelStepper.h>

#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(700);
  stepper.setAcceleration(700);
}

void loop() {
  // Calculate the number of steps for a 180-degree rotation (adjust this value if needed):
  long targetPosition = stepper.currentPosition() + stepsForDegrees(900); 
  //long targetPosition = stepsForDegrees(1800);
  
  // Set the target position:
  stepper.moveTo(targetPosition);
  stepper.runToPosition();
  Serial.println(stepper.currentPosition());
  targetPosition = stepsForDegrees(0);
  stepper.moveTo(targetPosition);
  stepper.runToPosition();

  delay(1000);

}

// Function to calculate the steps needed for a given number of degrees:
long stepsForDegrees(float degrees) {
  // Assuming a full step motor with 200 steps per revolution
  // Change this value if your motor has a different step count
  float stepsPerRevolution = 800.0;
  float degreesPerStep = 360.0 / stepsPerRevolution;
  return degrees / degreesPerStep;
}