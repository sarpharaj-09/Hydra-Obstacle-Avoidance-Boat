#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <NewPing.h>
#include "../config/PinConfig.h"
#include "../config/RobotConfig.h"

class UltrasonicManager {
public:
    UltrasonicManager();
    unsigned int getDistance(NewPing &sensor);
    unsigned int getFrontDistance();
    unsigned int getLeftDistance();
    unsigned int getRightDistance();
    void updateAllDistances();
    
    unsigned int frontDistance;
    unsigned int leftDistance;
    unsigned int rightDistance;

private:
    NewPing sonarFront;
    NewPing sonarLeft;
    NewPing sonarRight;
};

#endif