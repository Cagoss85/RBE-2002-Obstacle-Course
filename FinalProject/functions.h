#ifndef __FUNCTIONS_H
#define __FUNCTIONS_H
#include <line.h>

//Velocity PID
static float Kp = 10.75;
static float Ki = .875;
static float Kd = 0;

//Wall Following
static float Kp2 = 1.2;
static float Ki2 = 0;
static float Kd2 = 6;

//Line Follower PID
static float Kp3 = .0125;
static float Ki3 = 0;
static float Kd3 = .001;

//angle change detection
static float angleThresh = 10;

static float targetLeft = 0;      //target (ticks/time interval) for the left motor
static float targetRight = 0;     //target (ticks/time interval) for the right motor

static float sensorError = 0;     //difference between 2 line sensors
static float sumError = 0;        //sum of the error over time
static float lastSensorError = 0; //previous error from the last iteration

static float targetSpeed = 10;    //set the target speed (ticks/ time interval) for the robot
static float targetDist = 15;
/*Header Graveyard *//*_________________________________________________**
//bool pitchChange();

*/
#endif
