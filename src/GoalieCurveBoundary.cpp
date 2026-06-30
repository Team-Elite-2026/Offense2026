#include <GoalieCurveBoundary.h>

#include <math.h>

GoalieCurveBoundary::GoalieCurveBoundary(const GoalieCurveBoundaryConfig& config)
    : _config(config) {
}

namespace
{
constexpr double kMaxBoundaryBlendStrength = 0.65;

const char* stateName(GoalieCurveBoundaryState state)
{
    switch (state)
    {
        case GoalieCurveBoundaryState::TooClose: return "TOO_CLOSE";
        case GoalieCurveBoundaryState::TooFar: return "TOO_FAR";
        default: return "INSIDE";
    }
}
}

GoalieCurveBoundary::Candidate GoalieCurveBoundary::verticalCandidate(
    const Point& currentPose,
    double x,
    double minY,
    double maxY,
    double normalX,
    double normalY)
{
    double closestY = Trig::clamp(currentPose.y, minY, maxY);
    double dx = currentPose.x - x;
    double dy = currentPose.y - closestY;
    return {x, closestY, normalX, normalY, (dx * dx) + (dy * dy)};
}

GoalieCurveBoundary::Candidate GoalieCurveBoundary::horizontalCandidate(
    const Point& currentPose,
    double minX,
    double maxX,
    double y,
    double normalX,
    double normalY)
{
    double closestX = Trig::clamp(currentPose.x, minX, maxX);
    double dx = currentPose.x - closestX;
    double dy = currentPose.y - y;
    return {closestX, y, normalX, normalY, (dx * dx) + (dy * dy)};
}

GoalieCurveBoundary::Candidate GoalieCurveBoundary::arcCandidate(
    const Point& currentPose,
    double centerX,
    double centerY,
    double minTheta,
    double maxTheta)
{
    double dx = currentPose.x - centerX;
    double dy = currentPose.y - centerY;
    double theta = (fabs(dx) < 1e-6 && fabs(dy) < 1e-6)
        ? ((minTheta + maxTheta) * 0.5)
        : atan2(dy, dx);

    while (theta < 0.0)
    {
        theta += 2.0 * M_PI;
    }
    while (theta >= 2.0 * M_PI)
    {
        theta -= 2.0 * M_PI;
    }
    theta = Trig::clamp(theta, minTheta, maxTheta);

    double normalX = cos(theta);
    double normalY = sin(theta);
    double closestX = centerX + (kArcRadius * normalX);
    double closestY = centerY + (kArcRadius * normalY);
    double distX = currentPose.x - closestX;
    double distY = currentPose.y - closestY;
    return {closestX, closestY, normalX, normalY, (distX * distX) + (distY * distY)};
}

void GoalieCurveBoundary::keepClosest(const Candidate& candidate,
                                      Candidate& best,
                                      bool& hasBest)
{
    if (!hasBest || candidate.distanceSq < best.distanceSq)
    {
        best = candidate;
        hasBest = true;
    }
}

GoalieCurveBoundaryResult GoalieCurveBoundary::evaluate(
    const Point& currentPose,
    double robotHeadingDegrees) const
{
    Candidate best = {0.0, 0.0, 0.0, 1.0, 0.0};
    bool hasBest = false;

    keepClosest(verticalCandidate(currentPose, kLeftX, kSideVerticalEndY, kArcCenterY, -1.0, 0.0), best, hasBest);
    keepClosest(arcCandidate(currentPose, kLeftArcCenterX, kArcCenterY, M_PI * 0.5, M_PI), best, hasBest);
    keepClosest(horizontalCandidate(currentPose, kLeftArcCenterX, kRightArcCenterX, kGoalieLineY, 0.0, 1.0), best, hasBest);
    keepClosest(arcCandidate(currentPose, kRightArcCenterX, kArcCenterY, 0.0, M_PI * 0.5), best, hasBest);
    keepClosest(verticalCandidate(currentPose, kRightX, kSideVerticalEndY, kArcCenterY, 1.0, 0.0), best, hasBest);

    GoalieCurveBoundaryResult result;
    result.closestX = best.closestX;
    result.closestY = best.closestY;
    result.normalX = best.normalX;
    result.normalY = best.normalY;
    result.signedDistanceMm =
        ((currentPose.x - best.closestX) * best.normalX) +
        ((currentPose.y - best.closestY) * best.normalY);

    double correctionX = 0.0;
    double correctionY = 0.0;
    double error = 0.0;

    if (result.signedDistanceMm < _config.minOffsetMm)
    {
        result.state = GoalieCurveBoundaryState::TooClose;
        correctionX = best.normalX;
        correctionY = best.normalY;
        error = _config.minOffsetMm - result.signedDistanceMm;
    }
    else if (result.signedDistanceMm > _config.maxOffsetMm)
    {
        result.state = GoalieCurveBoundaryState::TooFar;
        correctionX = -best.normalX;
        correctionY = -best.normalY;
        error = result.signedDistanceMm - _config.maxOffsetMm;
    }

    result.hardRecovery =
        result.signedDistanceMm < _config.hardMinOffsetMm ||
        result.signedDistanceMm > _config.hardMaxOffsetMm;

    if (error <= 0.0)
    {
        return result;
    }

    double rampDistance = (_config.rampDistanceMm <= 0.0) ? 1.0 : _config.rampDistanceMm;
    result.correctionStrength = Trig::clamp(error / rampDistance, 0.0, kMaxBoundaryBlendStrength);
    result.correctionFieldAngle = Trig::normalize360(Trig::toDegrees(atan2(correctionX, correctionY)));
    result.correctionRobotAngle = Trig::normalize360(result.correctionFieldAngle - robotHeadingDegrees);
    return result;
}

double GoalieCurveBoundary::blendWithDefenseAngle(
    double defenseRobotAngle,
    double robotHeadingDegrees,
    double boundaryWeight,
    const GoalieCurveBoundaryResult& result) const
{
    double defenseFieldAngle = Trig::normalize360(defenseRobotAngle + robotHeadingDegrees);
    double weightedStrength = Trig::clamp(
        boundaryWeight * result.correctionStrength,
        0.0,
        kMaxBoundaryBlendStrength);
    double blendedX = Trig::Sin(defenseFieldAngle) +
                      (weightedStrength * Trig::Sin(result.correctionFieldAngle));
    double blendedY = Trig::Cos(defenseFieldAngle) +
                      (weightedStrength * Trig::Cos(result.correctionFieldAngle));

    if ((blendedX * blendedX + blendedY * blendedY) < 1e-6)
    {
        return Trig::normalize360(defenseRobotAngle);
    }

    double blendedFieldAngle = Trig::normalize360(Trig::toDegrees(atan2(blendedX, blendedY)));
    return Trig::normalize360(blendedFieldAngle - robotHeadingDegrees);
}

void GoalieCurveBoundary::printDebug(const GoalieCurveBoundaryResult& result,
                                     double blendedRobotAngle) const
{
    // Serial.println("Goalie Curve State: " + String(stateName(result.state)));
    // Serial.println("Goalie Curve Closest Point: " + String(result.closestX) + ", " + String(result.closestY));
    Serial.println("Goalie Curve Normal: " + String(result.normalX) + ", " + String(result.normalY));
    Serial.println("Goalie Curve Signed Distance: " + String(result.signedDistanceMm));
    Serial.println("Goalie Curve Correction Strength: " + String(result.correctionStrength));
    // Serial.println("Goalie Curve Correction Field Angle: " + String(result.correctionFieldAngle));
    // Serial.println("Goalie Curve Correction Robot Angle: " + String(result.correctionRobotAngle));
    Serial.println("Goalie Curve Blended Robot Angle: " + String(blendedRobotAngle));
}
