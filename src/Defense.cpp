#include <Defense.h>

#include <math.h>
#include <trig.h>

Defense::Defense() : defenseAngle(-1) {}

double Defense::normalize360(double angle)
{
    angle = fmod(angle, 360.0);
    if (angle < 0)
    {
        angle += 360.0;
    }
    return angle;
}

double Defense::normalize180(double angle)
{
    angle = normalize360(angle);
    if (angle > 180.0)
    {
        angle -= 360.0;
    }
    return angle;
}

double Defense::angularDistance(double a, double b)
{
    return fabs(normalize180(a - b));
}

double Defense::projectAngle(double lineReferenceAngle, double movementAngle)
{
    double lineAngle = normalize360(lineReferenceAngle + 180.0);
    if (angularDistance(movementAngle, lineAngle) > 90.0)
    {
        return normalize360(movementAngle);
    }

    double tangentAngle = normalize360(lineReferenceAngle + 90.0);
    double vectorX = Trig::Sin(tangentAngle);
    double vectorY = Trig::Cos(tangentAngle);
    double moveX = Trig::Sin(movementAngle);
    double moveY = Trig::Cos(movementAngle);

    double denominator = (vectorX * vectorX) + (vectorY * vectorY);
    if (denominator < 1e-6)
    {
        return normalize360(movementAngle);
    }

    double dot = ((moveX * vectorX) + (moveY * vectorY)) / denominator;
    double projectedX = dot * vectorX;
    double projectedY = dot * vectorY;

    double projectedAngle = Trig::toDegrees(atan2(projectedX, projectedY));
    return normalize360(projectedAngle);
}

double Defense::defenseCalc(double ballAngle, double homeGoalAngle, double headingCorrection)
{
    double ball = normalize360(ballAngle);
    double goal = normalize360(homeGoalAngle);
    double rotatedBall = normalize360(ball + headingCorrection);
    double rotatedGoal = normalize360(goal + headingCorrection);

    const double angleThreshold = 170.0;
    double angleDiff = angularDistance(ball, goal);

    bool blocked = (angleDiff > angleThreshold) ||
                   (rotatedGoal < 115.0 && rotatedBall > 115.0 && rotatedBall < 260.0) ||
                   (rotatedGoal > 245.0 && rotatedBall < 245.0 && rotatedBall > 100.0);

    if (blocked)
    {
        if (angleDiff > 170.0 && hardStop <= 100 && defenseAngle >= 0)
        {
            defenseAngle = normalize360(defenseAngle + 180.0);
        }
        else
        {
            hardStop = 200;
            defenseAngle = -1;
        }

        Serial.print("defense Angle: ");
        Serial.println(defenseAngle);
        return defenseAngle;
    }

    hardStop = 0;

    double robotAngleX = Trig::Sin(ball) + Trig::Sin(goal);
    double robotAngleY = Trig::Cos(ball) + Trig::Cos(goal);
    defenseAngle = normalize360(Trig::toDegrees(atan2(robotAngleX, robotAngleY)));

    double updatedHorizontal = normalize360(180.0 - headingCorrection);
    defenseAngle = projectAngle(updatedHorizontal, defenseAngle);

    Serial.print("defense Angle: ");
    Serial.println(defenseAngle);
    return defenseAngle;
}
