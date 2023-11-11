#include <AccelStepper.h>

#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(900);
  stepper.setAcceleration(700);
}

void loop() {
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
  float desiredAngle = input.toFloat();

  // Calculate the target position in steps
  long targetPosition = stepsForDegrees(desiredAngle);

  // Set the target position:
  stepper.moveTo(targetPosition);
  stepper.runToPosition();

  // Print the current position (in steps) and angle (in degrees)
  Serial.print("Current Position (steps): ");
  Serial.println(stepper.currentPosition());
  Serial.print("Current Angle (degrees): ");
  Serial.println(currentAngle());

  delay(100);
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
