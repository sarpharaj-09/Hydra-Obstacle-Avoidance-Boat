#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// Thresholds and settings
const unsigned long MIN_FORWARD_TIME = 10000;
const int MAX_DISTANCE_FRONT = 50;
const int MAX_DISTANCE_SIDE = 50;
const int TOLERANCE = 4;
const int CRITICAL_DISTANCE_SIDE = 5;
const int CRITICAL_DISTANCE_FRONT = 12;
const int THRESHOLD_DISTANCE_SIDE = 15;
const int THRESHOLD_DISTANCE_FRONT = 25;
const int MOTOR_SPEED_FORWARD = 255;
const int MOTOR_SPEED_REVERSE = 255;
const int MOTOR_SPEED_TURN = 180;
const int MOTOR_SPEED_DEFAULT_RIGHT = 100;
const int MOTOR_SPEED_DEFAULT_LEFT = 120;
const int OBSTACLE_CRITICAL_LEFT = 4;
const int OBSTACLE_CRITICAL_RIGHT = 4;
const int TURN_DELAY = 500;
const int TURN_DELAY_Curve = 500;
const int REVERSE_DELAY = 800;

#endif