#ifndef CAM_H
#define CAM_H

#include <Arduino.h>
#include <string>
#include <Adafruit_I2CDevice.h>
#include <iostream>
#include <trig.h>
#include <switches.h>
#include <map>

class Cam
{
public:
        Cam();
        double yellowGoal;
        double blueGoal;
        double ballAngle;
        double ballDist;
        double CamCalc();
        std::string buffer;
        bool inIntake;

private:
        char read;
        double previousBallAngle;
        double previousBlueAngle;
        double previousYellowAngle;
        double previousBallDistance;
        elapsedMillis derivativeSample;
        Switch switches;
};
#endif