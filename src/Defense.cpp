#include <Defense.h>

#include <math.h>
#include <LineDetection.h>
#include <trig.h>

Defense::Defense() : defenseAngle(-1) {}

bool Defense::shouldForceForwardLineRecovery(double lineAngleDeg, double chordLengthNorm)
{
    if (lineAngleDeg == -5 || chordLengthNorm < 0.0)
    {
        return false;
    }

    double frontOffset = fabs(Trig::wrapAngle(lineAngleDeg));
    return chordLengthNorm < LineDetection::kLineRecoveryChordThreshold &&
           frontOffset <= LineDetection::kLineRecoveryFrontHalfAngleDeg;
}

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

double Defense::projectAngle(double lineNormalAngle, double movementAngle)
{
    double tangentPlus = normalize360(lineNormalAngle + 90.0);
    double tangentMinus = normalize360(lineNormalAngle - 90.0);

    if (angularDistance(movementAngle, tangentPlus) <= angularDistance(movementAngle, tangentMinus))
    {
        return tangentPlus;
    }
    return tangentMinus;
}

double Defense::clamp01(double value)
{
    if (value < 0.0)
    {
        return 0.0;
    }
    if (value > 1.0)
    {
        return 1.0;
    }
    return value;
}

double Defense::blendTangentWithNormal(double tangentAngle,
                                       double lineNormalAngle,
                                       double chordLengthNormalized,
                                       bool crossLine)
{
    double chord = clamp01(chordLengthNormalized);

    // Close to a full chord means the robot is already centered on the line:
    // keep motion almost purely tangent in this case.
    double normalGain = (chord >= 0.92) ? 0.0 : (0.65 * (1.0 - chord));

    // Opposite of avoidanceAngle(): if crossLine is true, normal correction points
    // to lineNormal+180; otherwise it points to lineNormal.
    double lineCorrectionAngle = crossLine ? normalize360(lineNormalAngle + 180.0) : normalize360(lineNormalAngle);
    double blendedX = Trig::Sin(tangentAngle) + (normalGain * Trig::Sin(lineCorrectionAngle));
    double blendedY = Trig::Cos(tangentAngle) + (normalGain * Trig::Cos(lineCorrectionAngle));

    if ((blendedX * blendedX + blendedY * blendedY) < 1e-6)
    {
        return normalize360(tangentAngle);
    }

    return normalize360(Trig::toDegrees(atan2(blendedX, blendedY)));
}

double Defense::defenseCalc(double ballAngle,
                            double homeGoalAngle,
                            double headingCorrection,
                            double lineNormalAngle,
                            double chordLengthNormalized,
                            bool crossLine)
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

    if (lineNormalAngle < 0.0)
    {
        Serial.print("defense Angle (no line): ");
        Serial.println(defenseAngle);
        return defenseAngle;
    }

    double tangentAngle = projectAngle(lineNormalAngle, defenseAngle);
    defenseAngle = blendTangentWithNormal(tangentAngle, lineNormalAngle, chordLengthNormalized, crossLine);

    Serial.print("defense Angle: ");
    Serial.println(defenseAngle);
    return defenseAngle;
}
