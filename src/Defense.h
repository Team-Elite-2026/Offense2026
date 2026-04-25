#ifndef DEFENSE_H
#define DEFENSE_H

#include <Arduino.h>

class Defense {
public:
    Defense();
    double defenseCalc(double ballAngle, double homeGoalAngle, double headingCorrection);

private:
    double defenseAngle;
    elapsedMillis hardStop;

    static double normalize360(double angle);
    static double normalize180(double angle);
    static double angularDistance(double a, double b);
    static double projectAngle(double lineReferenceAngle, double movementAngle);
};

#endif
