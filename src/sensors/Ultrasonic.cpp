#include "Ultrasonic.h"

UltrasonicManager::UltrasonicManager()
    : sonarFront(TRIG_PIN, ECHO_PIN_FRONT, MAX_DISTANCE_FRONT),
      sonarLeft(TRIG_PIN, ECHO_PIN_LEFT, MAX_DISTANCE_SIDE),
      sonarRight(TRIG_PIN, ECHO_PIN_RIGHT, MAX_DISTANCE_SIDE) {}

// Function to get the distance from a sensor
unsigned int UltrasonicManager::getDistance(NewPing &sensor) {
  unsigned int distance = sensor.ping_median(5) / US_ROUNDTRIP_CM;
  if (distance == 0) {
    if(&sensor == &sonarFront){
    distance = MAX_DISTANCE_FRONT;
    }
    else{
      distance = MAX_DISTANCE_SIDE;
    }
  }
  return distance;
}

unsigned int UltrasonicManager::getFrontDistance() {
    frontDistance = getDistance(sonarFront);
    return frontDistance;
}

unsigned int UltrasonicManager::getLeftDistance() {
    leftDistance = getDistance(sonarLeft);
    return leftDistance;
}

unsigned int UltrasonicManager::getRightDistance() {
    rightDistance = getDistance(sonarRight);
    return rightDistance;
}

void UltrasonicManager::updateAllDistances() {
    frontDistance = getFrontDistance();
    leftDistance = getLeftDistance();
    rightDistance = getRightDistance();
}