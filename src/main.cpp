#include <Arduino.h>
#include "sensors/Ultrasonic.h"
#include "motors/MotorController.h"
#include "navigation/ObstacleAvoidance.h"

UltrasonicManager ultrasonic;
MotorController motors;
ObstacleAvoidance navigation(ultrasonic, motors);

void setup() {
  Serial.begin(9600);

  motors.begin();

  Serial.println("Robot initialized.");
}

void loop() {
  navigation.handleObstacleDetection();

  // Small delay to avoid spamming
  delay(100);
}