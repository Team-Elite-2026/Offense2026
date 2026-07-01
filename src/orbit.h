
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
    double kd = 0.3;           // derivative (predicted-ball) gain
    int physicalRobot;
    double behindDist = 11.0;  // cm behind the ball = dribbler contact offset (target point)
    double kTan       = 50.0;  // tangential go-around gain for the goal-aware orbit

    // ---- Orbit formula tuning (kept here so they are easy to find/tune) ----
    // Distance dampening: orbit offset ramps in as the ball nears, over 0..kDistanceScaleCm.
    static constexpr double kDistanceScaleCm = 150.0;
    static constexpr double kDampenCoeff     = 0.02;
    static constexpr double kDampenExp       = 4.5;
    // Goal-blind legacy orbit offset:
    //   min(kMaxOrbitOffsetDeg, kLegacyOrbitCoeff * exp(kLegacyOrbitExp * (ballAngle - kLegacyOrbitAngleOffsetDeg)))
    static constexpr double kLegacyOrbitCoeff          = 4.0;
    static constexpr double kLegacyOrbitExp            = 0.1;
    static constexpr double kLegacyOrbitAngleOffsetDeg = 30.0;
    static constexpr double kMaxOrbitOffsetDeg         = 90.0;
    // Derivative damping: only subtract the D-term once it exceeds this.
    static constexpr double kDerivativeTermThreshold = 3.0;
    // Goal-aware: break the unstable delta==180 tie when |delta| exceeds this.
    static constexpr double kDeltaTieThresholdDeg = 179.0;
};
#endif
