#ifndef CAM_H
#define CAM_H

#include <Arduino.h>
#include <string>
#include <Adafruit_I2CDevice.h>
#include <iostream>
#include <trig.h>
#include <switches.h>
#include <map>

class Movement;

class Cam
{
public:
        Cam();
        double yellowGoal;
        double blueGoal;
        double ballAngle;
        double ballDist;
        double derivative;
        int sampleTime;
        double CamCalc();
        void setMovement(Movement* movement);
        std::string buffer;
        bool inIntake;

private:
        char read;
        Movement* movement;
        double poseX;
        double poseY;
        double previousBallAngle;
        double previousBlueAngle;
        double previousYellowAngle;
        double previousBallDistance;
        elapsedMillis derivativeSample;
        Switch switches;
};
#endif
