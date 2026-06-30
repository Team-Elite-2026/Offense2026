
#ifndef ORBIT_H
#define ORBIT_H

#include <trig.h>

class Orbit
{

public:
    Orbit(int robotNum);
    double robotAngle;
    // Distance (cm) from the robot to the behind-the-ball target point, updated by
    // CalculateRobotAngle. Used to decelerate on approach so momentum does not
    // overshoot the shot line. -1 until the first computation.
    double distanceToTarget = -1;
    double CalculateRobotAngle(double ballAngle, double distance, double derivative, int sampleTime,
                               double goalAngle, bool aimingGoal);
    bool inOrientation;

private:
    int multiplier;
    int lastAngle;
    double kd;
    int physicalRobot;
    double behindDist = 11.0;  // cm behind the ball = dribbler contact offset (target point)
    double kTan       = 50.0;  // tangential go-around gain for the goal-aware orbit
};
#endif
