#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "../config/PinConfig.h"
#include "../config/RobotConfig.h"

class MotorController {
public:
    MotorController();
    void begin();
    
    // Basic movements
    void moveForward();
    void stopMotors();
    void turnLeft();
    void turnRight();
    
    // Advanced movements
    void turnLeftCurve();
    void turnRightCurve();
    void reverse();
    void reverseAndTurn(unsigned int distanceLeft, unsigned int distanceRight);
    void strongerCurveTurn(bool turnLeft);
    
    // State tracking
    bool isCurrentlyMovingForward();
    unsigned long getForwardStartTime();
    void resetForwardTimer();

    // Adjustable speeds for curves - DECLARE THESE FIRST
    unsigned int MOTOR_SPEED_CURVE_RIGHT;
    unsigned int MOTOR_SPEED_CURVE_LEFT;

private:
    void controlMotor(int enablePin, int in1, int in2, int speed, bool forward);
    
    // THEN DECLARE THESE
    unsigned long forwardStartTime;
    bool isMovingForward;  // Member variable
};

#endif