#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include "../sensors/Ultrasonic.h"
#include "../motors/MotorController.h"

class ObstacleAvoidance {
public:
    ObstacleAvoidance(UltrasonicManager& sensors, MotorController& motors);
    void handleObstacleDetection();
    void logSensorData(unsigned int front, unsigned int left, unsigned int right);

private:
    UltrasonicManager& ultrasonic;
    MotorController& motorController;
};

#endif