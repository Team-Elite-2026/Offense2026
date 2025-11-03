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

    double kp = 1;
    double ki = 0;
    double kd = 0.001;

    double kp2 = 0.12;
    double ki2 = 0;
    double kd2 = 0.005;


public:
    Movement(Motor& FLMotor, Motor& FRMotor, Motor& BLMotor, Motor& BRMotor, CompassSensor& compassSensor);
    void movement(double intended_movement_angle, double speedfactor, double desiredOrientation);
    double findCorrection(double goalDirection);
    double CorrectionAngle();
    double goalCorrection(double goalDirection);
    void rotateToGoal(double goalDirection, double speedFactor);
    void circle();
    void stop();
    

};

#endif
