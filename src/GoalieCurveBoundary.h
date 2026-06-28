#ifndef GOALIE_CURVE_BOUNDARY_H
#define GOALIE_CURVE_BOUNDARY_H

#include <Arduino.h>
#include <trig.h>

enum class GoalieCurveBoundaryState {
    InsideBand,
    TooClose,
    TooFar
};

struct GoalieCurveBoundaryConfig {
    double minOffsetMm;
    double maxOffsetMm;
    double rampDistanceMm;
    double hardMinOffsetMm;
    double hardMaxOffsetMm;
};

struct GoalieCurveBoundaryResult {
    GoalieCurveBoundaryState state = GoalieCurveBoundaryState::InsideBand;
    bool hardRecovery = false;
    double closestX = 0.0;
    double closestY = 0.0;
    double normalX = 0.0;
    double normalY = 1.0;
    double signedDistanceMm = 0.0;
    double correctionStrength = 0.0;
    double correctionFieldAngle = -1.0;
    double correctionRobotAngle = -1.0;

    bool hasCorrection() const { return correctionStrength > 0.0; }
};

class GoalieCurveBoundary {
public:
    explicit GoalieCurveBoundary(const GoalieCurveBoundaryConfig& config);

    GoalieCurveBoundaryResult evaluate(const Point& currentPose,
                                       double robotHeadingDegrees) const;
    double blendWithDefenseAngle(double defenseRobotAngle,
                                 double robotHeadingDegrees,
                                 double boundaryWeight,
                                 const GoalieCurveBoundaryResult& result) const;
    void printDebug(const GoalieCurveBoundaryResult& result,
                    double blendedRobotAngle) const;

private:
    struct Candidate {
        double closestX;
        double closestY;
        double normalX;
        double normalY;
        double distanceSq;
    };

    GoalieCurveBoundaryConfig _config;

    static constexpr double kLeftX = -390.0;
    static constexpr double kRightX = 390.0;
    static constexpr double kBottomY = -1075.0;
    static constexpr double kArcCenterY = -975.0;
    static constexpr double kTopY = -835.0;
    static constexpr double kLeftArcCenterX = -250.0;
    static constexpr double kRightArcCenterX = 250.0;
    static constexpr double kArcRadius = 140.0;

    static Candidate verticalCandidate(const Point& currentPose,
                                       double x,
                                       double minY,
                                       double maxY,
                                       double normalX,
                                       double normalY);
    static Candidate horizontalCandidate(const Point& currentPose,
                                         double minX,
                                         double maxX,
                                         double y,
                                         double normalX,
                                         double normalY);
    static Candidate arcCandidate(const Point& currentPose,
                                  double centerX,
                                  double centerY,
                                  double minTheta,
                                  double maxTheta);
    static void keepClosest(const Candidate& candidate,
                            Candidate& best,
                            bool& hasBest);
};

#endif
