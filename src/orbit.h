
#ifndef ORBIT_H
#define ORBIT_H

#include <trig.h>

class Orbit
{

public:
    Orbit(int robotNum);
    double robotAngle;
    double CalculateRobotAngle(double ballAngle, double distance, double derivative, int sampleTime);
    double GetToPosition(int targetX, int targetY, int currentX, int currentY);
    bool inOrientation;

private:
    int multiplier;
    double homeAngle;
    int lastAngle;
    double kd;
    int physicalRobot;
};
#endif
