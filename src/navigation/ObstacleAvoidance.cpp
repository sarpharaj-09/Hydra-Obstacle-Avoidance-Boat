#include "navigation/ObstacleAvoidance.h"
#include <Arduino.h>

ObstacleAvoidance::ObstacleAvoidance(UltrasonicManager& sensors, MotorController& motors)
    : ultrasonic(sensors), motorController(motors) {}

// Handles obstacle detection and navigation
void ObstacleAvoidance::handleObstacleDetection() {
  unsigned int distanceFront = ultrasonic.getFrontDistance();
  unsigned int distanceLeft = ultrasonic.getLeftDistance();
  unsigned int distanceRight = ultrasonic.getRightDistance();

  logSensorData(distanceFront, distanceLeft, distanceRight);

  int T = distanceLeft - distanceRight;

 if ((distanceFront < CRITICAL_DISTANCE_FRONT)) {
     motorController.reverseAndTurn(distanceLeft, distanceRight);
    motorController.resetForwardTimer();
  } 
  else if (distanceFront <= THRESHOLD_DISTANCE_FRONT) {
    motorController.resetForwardTimer();
    if (distanceLeft > THRESHOLD_DISTANCE_SIDE && distanceRight > THRESHOLD_DISTANCE_SIDE) {
      if (abs(T) <= TOLERANCE) {
        motorController.turnLeft();
      } else if (distanceLeft > distanceRight) {
        motorController.turnLeft();
      } else {
        motorController.turnRight();
      }
    } else if (distanceLeft > THRESHOLD_DISTANCE_SIDE) {
      motorController.turnLeft();
    } else if (distanceRight > THRESHOLD_DISTANCE_SIDE) {
      motorController.turnRight();
    } else {
      motorController.reverseAndTurn(distanceLeft, distanceRight);
    }
  } 
else {
    if (abs(T) <= TOLERANCE) {
        motorController.moveForward();
    } 
    else if ((distanceLeft < OBSTACLE_CRITICAL_LEFT) || (distanceRight < OBSTACLE_CRITICAL_RIGHT)) {
        motorController.MOTOR_SPEED_CURVE_RIGHT = MOTOR_SPEED_DEFAULT_RIGHT;
        motorController.MOTOR_SPEED_CURVE_LEFT = MOTOR_SPEED_DEFAULT_LEFT;

        if (distanceLeft < OBSTACLE_CRITICAL_RIGHT) {
            motorController.turnRightCurve();
        } else {
            motorController.turnLeftCurve();
        }
    } 
    // FIXED: Updated method name
    else if (motorController.isCurrentlyMovingForward() && ((millis() - motorController.getForwardStartTime()) > MIN_FORWARD_TIME)) {
        motorController.resetForwardTimer();

        if (distanceLeft > distanceRight) {
            motorController.strongerCurveTurn(true);
        } else {
            motorController.strongerCurveTurn(false);
        }
    } 
    else if (distanceLeft > distanceRight) {
        motorController.turnLeftCurve();
    } else {
        motorController.turnRightCurve();
    }
}
}

// Logs sensor data for debugging
void ObstacleAvoidance::logSensorData(unsigned int front, unsigned int left, unsigned int right) {
  Serial.print("Front: ");
  Serial.print(front);
  Serial.print(" cm, Left: ");
  Serial.print(left);
  Serial.print(" cm, Right: ");
  Serial.println(right);
}


