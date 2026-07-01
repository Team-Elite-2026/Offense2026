#include <Defense.h>

#include <math.h>
#include <trig.h>

Defense::Defense() : defenseAngle(-1), lastTangentAngle(-1) {}

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
    double normalGain = (chord >= kChordCenteredThreshold) ? 0.0 : (kNormalBlendGainMax * (1.0 - chord));

    // Opposite of avoidanceAngle(): if crossLine is true, normal correction points
    // to lineNormal+180; otherwise it points to lineNormal.
    
    double lineCorrectionAngle = crossLine ? Trig::normalize360(lineNormalAngle + 180.0) : Trig::normalize360(lineNormalAngle);

    // Blend the tangent with the normal correction, weighted by normalGain.
    return Trig::blendAngles(tangentAngle, lineCorrectionAngle, normalGain);
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

    double angleDiff = Trig::angularDistance(ball, goal);

    bool blocked = (angleDiff > kBlockedAngleThresholdDeg) ||
                   (rotatedGoal < kBlockedRightGoalMaxDeg && rotatedBall > kBlockedRightBallMinDeg && rotatedBall < kBlockedRightBallMaxDeg) ||
                   (rotatedGoal > kBlockedLeftGoalMinDeg && rotatedBall < kBlockedLeftBallMaxDeg && rotatedBall > kBlockedLeftBallMinDeg);

    if (blocked)
    {
        if (angleDiff > kBlockedAngleThresholdDeg && hardStop <= kHardStopRecoverMs && defenseAngle >= 0)
        {
            defenseAngle = clampDefenseMoveAngle(defenseAngle + 180.0);
        }
        else
        {
            hardStop = kHardStopHoldMs;
            defenseAngle = -1;
        }

        return defenseAngle;
    }

    hardStop = 0;

    defenseAngle = Trig::bisectAngles(ball, goal);

    if (lineNormalAngle < 0.0)
    {
        lastTangentAngle = -1;
        defenseAngle = clampDefenseMoveAngle(defenseAngle);
        Serial.print("defense Angle (no line): ");
        Serial.println(defenseAngle);
        return defenseAngle;
    }

    double tangentAngle = Trig::projectTangent(lineNormalAngle, defenseAngle);
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
