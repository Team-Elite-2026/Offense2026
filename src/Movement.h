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

    double max_power; 
    double Setpoint, Input, Output;

    elapsedMillis timer;
    elapsedMillis active;
    int kickerPin = 30;
    int kickHold = 1000;


public:
    Movement(Motor& FLMotor, Motor& FRMotor, Motor& BLMotor, Motor& BRMotor, CompassSensor& compassSensor);
    void movement(double intended_movement_angle, double speedfactor, double desiredOrientation, bool AimingGoal);
    double findCorrectionRelZero(double goalDirection);
    double findCorrectionRelOffset(double goalDirection);
    double CorrectionAngle();
    double goalCorrection(double goalDirection);
    void rotateToGoal(double goalDirection, double speedFactor);
    void circle();
    void stop();
    void kick();
    void kickBackground();


    double kp = 0.6;
    double ki = 0;
    double kd = 0.000005;

    PID* myPID;

    
    

};

#endif
