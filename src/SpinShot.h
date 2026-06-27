#ifndef SPIN_SHOT_H
#define SPIN_SHOT_H
#include <trig.h>
#include <math.h>

class SpinShot {
    public:
        int driveAngle(bool hasBall);
        int goToPointAngle(Point ourPosition, Point targetPosition); // Returns an angle to travel to point -- Returns [0-360)
        int driveHeading();
        bool activateDribbler();

};
#endif