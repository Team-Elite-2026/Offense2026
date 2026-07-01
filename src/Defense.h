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

    // ---- defenseCalc tuning (kept here so they are easy to find/tune) ----
    // Ball and goal this far apart (deg) means the ball is behind us -> blocked.
    static constexpr double kBlockedAngleThresholdDeg = 170.0;
    // "Blocked" angular sectors in the heading-rotated frame (deg): goal on one
    // side of the robot while the ball is on the other.
    static constexpr double kBlockedRightGoalMaxDeg = 115.0;
    static constexpr double kBlockedRightBallMinDeg = 115.0;
    static constexpr double kBlockedRightBallMaxDeg = 260.0;
    static constexpr double kBlockedLeftGoalMinDeg  = 245.0;
    static constexpr double kBlockedLeftBallMaxDeg  = 245.0;
    static constexpr double kBlockedLeftBallMinDeg  = 100.0;
    // hardStop dwell: allow the +180 recovery only while below kHardStopRecoverMs;
    // when freshly blocked, hold for kHardStopHoldMs.
    static constexpr unsigned long kHardStopRecoverMs = 100;
    static constexpr unsigned long kHardStopHoldMs    = 200;

    // ---- blendTangentWithNormal tuning ----
    // chord >= this means we are centred on the line -> drive purely tangent.
    static constexpr double kChordCenteredThreshold = 0.92;
    // Weight of the normal correction at zero chord (scales down to 0 as chord->1).
    static constexpr double kNormalBlendGainMax     = 0.65;

private:
    double defenseAngle;
    double lastTangentAngle;
    double sidewaysHeadingTolerance = 7.0;
    elapsedMillis hardStop;

    static double clampDefenseMoveAngle(double movementAngle);
    static double blendTangentWithNormal(double tangentAngle,
                                         double lineNormalAngle,
                                         double chordLengthNormalized,
                                         bool crossLine);
};

#endif
