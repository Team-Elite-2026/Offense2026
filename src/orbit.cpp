#include <orbit.h>
#include <math.h>
#include <trig.h>
#include <Arduino.h>

Orbit::Orbit(int robotNum)
{
    physicalRobot = robotNum;
    kd = 0.3;
}

double Orbit::CalculateRobotAngle(double ballAngle, double distance, double derivative, int sampleTime,
                                  double goalAngle, bool aimingGoal)
{
    double dTerm = 0;
    if (derivative != -5 && sampleTime > 0)
    {
        double sampleTimeInSec = static_cast<double>(sampleTime) / 1000.0;
        dTerm = kd * (derivative / sampleTimeInSec);
    }

    // Preserve the raw distance in cm: the goal-aware target-point math below needs
    // the true distance, while the legacy dampening reuses `distance` as a 0..1 value.
    double rawDistance = distance;

    distance = distance / 150;
    if (distance > 1)
    {
        distance = 1;
    }
    distance = 1 - distance;
    // Serial.print("calculated distance: ");
    // Serial.println(distance);
    // double dampenVal = min(1, 0.025 * exp(4.5 * distance));
    double dampenVal = Trig::min(1, 0.02 * exp(4.5 * distance));
    // Serial.print("dampen val: ");
    // Serial.println(dampenVal);

    // offset is the angle added to the ball bearing to produce the movement
    // command; how we compute it depends on whether the goal is in view.
    double offset;

    if (!aimingGoal)
    {
        // ---- Goal not in view: fall back to the original goal-blind orbit ----
        // takes absolute value of ball angle from 0 - 180 range
        double newballAngle = ballAngle > 180 ? (360 - ballAngle) : ballAngle;
        double orbitValue = Trig::min(90, 4 * exp(0.1 * (newballAngle - 30)));

        double outputSum = orbitValue * dampenVal;
        if (dTerm > 3)
        {
            outputSum -= dTerm;
        }
        offset = (ballAngle > 180 ? -1 : 1) * outputSum;

        // No goal target available; decelerate on raw ball distance instead.
        distanceToTarget = rawDistance;
    }
    else
    {
        // ---- Goal-aware orbit ----
        // Steer toward the "target point" a fixed distance (behindDist) BEHIND the
        // ball on the ball->goal line, so at contact robot->ball->goal are colinear
        // and the ball is pushed straight at the goal.  All angles are robot-relative;
        // ball->goal direction is approximated by goalAngle (exact as we reach the ball).
        //
        //   T = ballVec - behindDist * unit(goalAngle)
        //   coreOffset = bearing(T) - ballAngle
        double tx = rawDistance * Trig::Sin(ballAngle) - behindDist * Trig::Sin(goalAngle);
        double ty = rawDistance * Trig::Cos(ballAngle) - behindDist * Trig::Cos(goalAngle);
        double coreOffset = Trig::normalize180(Trig::toDegrees(atan2(tx, ty)) - ballAngle);

        // Distance to the behind-the-ball target; drives the approach deceleration.
        distanceToTarget = sqrt(tx * tx + ty * ty);

        // Tangential go-around term: when we are on the goal side of the ball the
        // target-point core alone would drive through the ball, so add a push that
        // commits us to arc onto the anti-goal side.  It decays with distance using
        // the same dampening as the legacy orbit (aggressive far out, gentle close in).
        double delta = Trig::normalize180(goalAngle - ballAngle);
        double boost = -kTan * Trig::Sin(delta) * dampenVal;
        if (fabs(delta) > 179)
        {
            boost = -kTan * dampenVal;  // break the unstable delta == 180 tie
        }

        offset = coreOffset + boost;
    }

    robotAngle = Trig::normalize360(ballAngle + offset);
    // Serial.print("robot Angle: ");
    // Serial.println(robotAngle);
    return robotAngle;
}
