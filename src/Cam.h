#ifndef CAM_H
#define CAM_H

#include <Arduino.h>
#include <string>
#include <Adafruit_I2CDevice.h>
#include <iostream>
#include <trig.h>
#include <map>

class Movement;
class ModeControl;

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
        void setModeControl(ModeControl* modeControl);
        std::string buffer;
        bool inIntake;

private:
        char read;
        Movement* movement;
        ModeControl* modeControl;
        double poseX;
        double poseY;
        double previousBallAngle;
        double previousBlueAngle;
        double previousYellowAngle;
        double previousBallDistance;
        elapsedMillis derivativeSample;
};
#endif
