#include "MotorController.h"
#include <Arduino.h>

// FIXED: Initialization order NOW MATCHES declaration order
MotorController::MotorController() 
    : MOTOR_SPEED_CURVE_RIGHT(MOTOR_SPEED_DEFAULT_RIGHT),  // 1st declared
      MOTOR_SPEED_CURVE_LEFT(MOTOR_SPEED_DEFAULT_LEFT),    // 2nd declared
      forwardStartTime(0),                                 // 3rd declared
      isMovingForward(false) {}                            // 4th declared

void MotorController::begin() {
  pinMode(MOTOR_LEFT_ENABLE, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_ENABLE, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
}
// General motor control function
void MotorController::controlMotor(int enablePin, int in1, int in2, int speed, bool forward) {
  digitalWrite(in1, forward ? LOW : HIGH);
  digitalWrite(in2, forward ? HIGH : LOW);
  analogWrite(enablePin, speed);
}

// Moves the robot forward
void MotorController::moveForward() {
  if (!isMovingForward) {
    forwardStartTime = millis();
    isMovingForward = true;
  }
  
  controlMotor(MOTOR_LEFT_ENABLE, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_SPEED_FORWARD, true);
  controlMotor(MOTOR_RIGHT_ENABLE, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_SPEED_FORWARD, true);
  Serial.println("Moving forward.");
}

// Stops the robot
void MotorController::stopMotors() {
  analogWrite(MOTOR_LEFT_ENABLE, 0);
  analogWrite(MOTOR_RIGHT_ENABLE, 0);
  Serial.println("Motors stopped.");
}

// Turns the robot left
void MotorController::turnLeft() {
  controlMotor(MOTOR_LEFT_ENABLE, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_SPEED_TURN, false);
  controlMotor(MOTOR_RIGHT_ENABLE, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_SPEED_TURN, true);
  delay(TURN_DELAY);
  stopMotors();
  Serial.println("Turning left.");
}

// Turns the robot right
void MotorController::turnRight() {
  controlMotor(MOTOR_LEFT_ENABLE, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_SPEED_TURN, true);
  controlMotor(MOTOR_RIGHT_ENABLE, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_SPEED_TURN, false);
  delay(TURN_DELAY);
  stopMotors();
  Serial.println("Turning right.");
}

// Executes a left curve
void MotorController::turnLeftCurve() {
  unsigned long startTime = millis();
  while ((millis() - startTime) < TURN_DELAY_Curve) {
    controlMotor(MOTOR_LEFT_ENABLE, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_SPEED_CURVE_LEFT, true);
    controlMotor(MOTOR_RIGHT_ENABLE, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_SPEED_FORWARD, true);
  }
  stopMotors();
  Serial.println("Completed left curve.");
}

// Executes a right curve
void MotorController::turnRightCurve() {
  unsigned long startTime = millis();
  while ((millis() - startTime) < TURN_DELAY_Curve) {
    controlMotor(MOTOR_LEFT_ENABLE, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_SPEED_FORWARD, true);
    controlMotor(MOTOR_RIGHT_ENABLE, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_SPEED_CURVE_RIGHT, true);
  }
  stopMotors();
  Serial.println("Completed right curve.");
}

void MotorController::strongerCurveTurn(bool turnLeft) {
  Serial.println(turnLeft ? "Executing stronger left curve." : "Executing stronger right curve.");

  if (turnLeft) {
    controlMotor(MOTOR_LEFT_ENABLE, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_SPEED_TURN, false);
    controlMotor(MOTOR_RIGHT_ENABLE, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_SPEED_FORWARD, true);
  } else {
    controlMotor(MOTOR_LEFT_ENABLE, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_SPEED_FORWARD, true);
    controlMotor(MOTOR_RIGHT_ENABLE, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_SPEED_TURN, false);
  }
  
  delay(100);
  stopMotors();
}

void MotorController::reverse(){
  controlMotor(MOTOR_LEFT_ENABLE, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_SPEED_TURN, false);
  controlMotor(MOTOR_RIGHT_ENABLE, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_SPEED_TURN, false);
  delay(REVERSE_DELAY);
}

// Reverses and then turns
void MotorController::reverseAndTurn(unsigned int distanceLeft, unsigned int distanceRight) {
  stopMotors();
  delay(100);
  reverse();
  delay(REVERSE_DELAY);

  int T = distanceLeft - distanceRight;

  if (abs(T) <= TOLERANCE) {
     turnLeft();
     delay(100);
  } 
  else if (distanceLeft > distanceRight) {
    turnLeft();
     delay(100);
    Serial.println("Reversing and turning left.");
  } 
  else {
    turnRight();
     delay(100);
    Serial.println("Reversing and turning right.");
  }
}

// FIXED: Renamed method
bool MotorController::isCurrentlyMovingForward() {
    return isMovingForward;
}

unsigned long MotorController::getForwardStartTime() {
    return forwardStartTime;
}

void MotorController::resetForwardTimer() {
    isMovingForward = false;
}