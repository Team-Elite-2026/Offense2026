
#ifndef ORBIT_H
#define ORBIT_H

#include <trig.h>

class Orbit
{

public:
    Orbit(int robotNum);
    double robotAngle;
    double CalculateRobotAngle(double ballAngle, double distance, double derivative, int sampleTime);
    bool inOrientation;

private:
    int multiplier;
    int lastAngle;
    double kd;
    int physicalRobot;
};
#endif
