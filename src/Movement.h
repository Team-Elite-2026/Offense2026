#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Motor.h>
#include <CompassSensor.h>
#include <PID_v1.h>

class Movement {
private:
    Motor FLMotor;
    Motor FRMotor;
    Motor BLMotor;
    Motor BRMotor;

    CompassSensor& compassSensor;
    PID* myPID;
    PID* myPID2;

    double max_power; 
    double Setpoint, Input, Output;
    double Setpoint2, Input2, Output2;

    double kp = 0.3;
    double ki = 0;
    double kd = 0.000005;

    int kickerHold = 1000;
    elapsedMillis timer;
    elapsedMillis active;
    int kickerPin = 30;
    int kickHold = 1000;


public:
    Movement(Motor& FLMotor, Motor& FRMotor, Motor& BLMotor, Motor& BRMotor, CompassSensor& compassSensor);
    void movement(double intended_movement_angle, double speedfactor, double desiredOrientation);
    double findCorrection(double goalDirection);
    double CorrectionAngle();
    double goalCorrection(double goalDirection);
    void rotateToGoal(double goalDirection, double speedFactor);
    void circle();
    void stop();
    void kick();
    void kickBackground();
    
    

};

#endif
