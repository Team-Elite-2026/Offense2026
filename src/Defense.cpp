#include <Defense.h>

#include <math.h>
#include <trig.h>

Defense::Defense() : defenseAngle(-1), lastTangentAngle(-1) {}

double Defense::projectAngle(double lineNormalAngle, double movementAngle)
{
    double tangentPlus = Trig::normalize360(lineNormalAngle + 90.0);
    double tangentMinus = Trig::normalize360(lineNormalAngle - 90.0);

    if (Trig::angularDistance(movementAngle, tangentPlus) <= Trig::angularDistance(movementAngle, tangentMinus))
    {
        return tangentPlus;
    }
    return tangentMinus;
}

double Defense::clampDefenseMoveAngle(double movementAngle)
{
    double angle = Trig::normalize360(movementAngle);
    double rightMin = kRightSlideAngle - kSlideAngleHalfWidth;
    double rightMax = kRightSlideAngle + kSlideAngleHalfWidth;
    double leftMin = kLeftSlideAngle - kSlideAngleHalfWidth;
    double leftMax = kLeftSlideAngle + kSlideAngleHalfWidth;

    if (Trig::angularDistance(angle, kRightSlideAngle) <= Trig::angularDistance(angle, kLeftSlideAngle))
    {
        if (angle < rightMin)
        {
            return rightMin;
        }
        if (angle > rightMax)
        {
            return rightMax;
        }
        return angle;
    }

    if (angle < leftMin)
    {
        return leftMin;
    }
    if (angle > leftMax)
    {
        return leftMax;
    }
    return angle;
}

double Defense::blendTangentWithNormal(double tangentAngle,
                                       double lineNormalAngle,
                                       double chordLengthNormalized,
                                       bool crossLine)
{
    double chord = Trig::clamp(chordLengthNormalized, 0.0, 1.0);

    // Close to a full chord means the robot is already centered on the line:
    // keep motion almost purely tangent in this case.
    double normalGain = (chord >= 0.92) ? 0.0 : (0.65 * (1.0 - chord));

    // Opposite of avoidanceAngle(): if crossLine is true, normal correction points
    // to lineNormal+180; otherwise it points to lineNormal.

    double lineCorrectionAngle = Trig::normalize360(lineNormalAngle) < 90 || Trig::normalize360(lineNormalAngle) > 270? Trig::normalize360(lineNormalAngle + 180.0) : Trig::normalize360(lineNormalAngle);
    double blendedX = Trig::Sin(tangentAngle) + (normalGain * Trig::Sin(lineCorrectionAngle));
    double blendedY = Trig::Cos(tangentAngle) + (normalGain * Trig::Cos(lineCorrectionAngle));

    if ((blendedX * blendedX + blendedY * blendedY) < 1e-6)
    {
        return Trig::normalize360(tangentAngle);
    }

    return Trig::normalize360(Trig::toDegrees(atan2(blendedX, blendedY)));
}

double Defense::lineFollowMoveAngle(double desiredMoveAngle,
                                    double lineNormalAngle,
                                    double chordLengthNormalized,
                                    bool crossLine)
{
    if (lineNormalAngle < 0.0)
    {
        return Trig::normalize360(desiredMoveAngle);
    }

    double tangentAngle = projectAngle(lineNormalAngle, desiredMoveAngle);
    return blendTangentWithNormal(
        tangentAngle,
        lineNormalAngle,
        chordLengthNormalized,
        crossLine);
}

double Defense::defenseCalc(double ballAngle,
                            double homeGoalAngle,
                            double headingCorrection,
                            double lineNormalAngle,
                            double chordLengthNormalized,
                            bool crossLine)
{
    double ball = Trig::normalize360(ballAngle);
    double goal = Trig::normalize360(homeGoalAngle);
    double rotatedBall = Trig::normalize360(ball + headingCorrection);
    double rotatedGoal = Trig::normalize360(goal + headingCorrection);

    const double angleThreshold = 170.0;
    double angleDiff = Trig::angularDistance(ball, goal);

    bool blocked = (angleDiff > angleThreshold) ||
                   (rotatedGoal < 115.0 && rotatedBall > 115.0 && rotatedBall < 260.0) ||
                   (rotatedGoal > 245.0 && rotatedBall < 245.0 && rotatedBall > 100.0);

    if (blocked)
    {
        if (angleDiff > 170.0 && hardStop <= 100 && defenseAngle >= 0)
        {
            defenseAngle = clampDefenseMoveAngle(defenseAngle + 180.0);
        }
        else
        {
            hardStop = 200;
            defenseAngle = -1;
        }

        return defenseAngle;
    }

    hardStop = 0;

    double robotAngleX = Trig::Sin(ball) + Trig::Sin(goal);
    double robotAngleY = Trig::Cos(ball) + Trig::Cos(goal);
    defenseAngle = Trig::normalize360(Trig::toDegrees(atan2(robotAngleX, robotAngleY)));

    if (lineNormalAngle < 0.0)
    {
        lastTangentAngle = -1;
        defenseAngle = clampDefenseMoveAngle(defenseAngle);
        Serial.print("defense Angle (no line): ");
        Serial.println(defenseAngle);
        return defenseAngle;
    }

    double tangentAngle = projectAngle(lineNormalAngle, defenseAngle);
    // bool sidewaysHeading = fabs(fabs(normalize180(headingCorrection)) - 70.0) <= sidewaysHeadingTolerance;

    // if (sidewaysHeading)
    // {
    //     if (lastTangentAngle < 0.0)
    //     {
    //         lastTangentAngle = tangentAngle;
    //     }
    //     tangentAngle = normalize360(lastTangentAngle + 180.0);
    // }
    // else
    // {
    //     lastTangentAngle = tangentAngle;
    // }

    defenseAngle = blendTangentWithNormal(tangentAngle, lineNormalAngle, chordLengthNormalized, crossLine);
    defenseAngle = clampDefenseMoveAngle(defenseAngle);

    Serial.print("defense Angle: ");
    Serial.println(defenseAngle);
    return defenseAngle;
}
