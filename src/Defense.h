#ifndef DEFENSE_H
#define DEFENSE_H

#include <Arduino.h>

class Defense {
public:
    static constexpr double kRightSlideAngle = 90.0;
    static constexpr double kLeftSlideAngle = 270.0;
    static constexpr double kSlideAngleHalfWidth = 10.0;

    Defense();
    double defenseCalc(double ballAngle,
                       double homeGoalAngle,
                       double headingCorrection,
                       double lineNormalAngle,
                       double chordLengthNormalized,
                       bool crossLine);
    double lineFollowMoveAngle(double desiredMoveAngle,
                               double lineNormalAngle,
                               double chordLengthNormalized,
                               bool crossLine);

private:
    double defenseAngle;
    double lastTangentAngle;
    double sidewaysHeadingTolerance = 7.0;
    elapsedMillis hardStop;

    static double projectAngle(double lineNormalAngle, double movementAngle);
    static double clampDefenseMoveAngle(double movementAngle);
    static double blendTangentWithNormal(double tangentAngle,
                                         double lineNormalAngle,
                                         double chordLengthNormalized,
                                         bool crossLine);
};

#endif
