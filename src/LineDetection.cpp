#include <LineDetection.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
#include <trig.h>

namespace {

constexpr double kNoLineAngle = -5.0;
constexpr double kIdealRingRadius = 89.0;
constexpr double kSensorTriggerOffset = 30.0;
constexpr double kVectorEpsilon = 1e-6;
constexpr double kCrossToggleThresholdDegrees = 90.0;

double normalize360(double angle) {
    while (angle < 0.0) {
        angle += 360.0;
    }
    while (angle >= 360.0) {
        angle -= 360.0;
    }
    return angle;
}

double angleFromXY(double x, double y) {
    return normalize360(std::atan2(x, y) * 180.0 / M_PI);
}

double circularDistanceDegrees(double a, double b) {
    double diff = std::fabs(normalize360(a) - normalize360(b));
    return std::min(diff, 360.0 - diff);
}

double headingDeltaDegrees(double previousHeading, double currentHeading) {
    return Trig::wrapAngle(previousHeading - currentHeading);
}

double pointMagnitudeSquared(const Point& point) {
    return (point.x * point.x) + (point.y * point.y);
}

Point calculatePrincipalComponent(double cov_xx, double cov_xy, double cov_yy) {
    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;
    double discriminant = trace * trace - (4.0 * det);

    if (discriminant < 0.0) {
        return {1.0, 0.0};
    }

    double sqrtDiscriminant = std::sqrt(discriminant);
    double lambda1 = (trace + sqrtDiscriminant) / 2.0;
    double lambda2 = (trace - sqrtDiscriminant) / 2.0;
    double lambda = (std::fabs(lambda1) > std::fabs(lambda2)) ? lambda1 : lambda2;

    Point eigenvec;
    double aMinusLambda = cov_xx - lambda;

    if (std::fabs(cov_xy) > kVectorEpsilon) {
        eigenvec.x = 1.0;
        eigenvec.y = -aMinusLambda / cov_xy;
    } else if (std::fabs(cov_yy - lambda) > kVectorEpsilon) {
        eigenvec.y = 1.0;
        eigenvec.x = -cov_xy / (cov_yy - lambda);
    } else {
        eigenvec.x = 1.0;
        eigenvec.y = 0.0;
    }

    double magnitude = std::sqrt((eigenvec.x * eigenvec.x) + (eigenvec.y * eigenvec.y));
    if (magnitude > kVectorEpsilon) {
        eigenvec.x /= magnitude;
        eigenvec.y /= magnitude;
    }

    return eigenvec;
}

int countTriggeredGroups(const int activatedVals[48]) {
    int groups = 0;
    bool previousTriggered = false;

    for (int i = 0; i < 48; ++i) {
        bool triggered = (activatedVals[i] == 1);
        if (triggered && !previousTriggered) {
            groups++;
        }
        previousTriggered = triggered;
    }

    if (groups > 1 && activatedVals[0] == 1 && activatedVals[47] == 1) {
        groups--;
    }

    return groups;
}

bool buildOppositePairVector(const std::vector<int>& pos, Point& pairVector) {
    if (pos.size() < 2) {
        return false;
    }

    int bestI = -1;
    int bestJ = -1;
    double bestDot = 1.0;

    for (size_t i = 0; i < pos.size(); ++i) {
        const Point& p1 = LineDetection::points[pos[i]];
        double x1 = p1.x / LineDetection::magnitudes[pos[i]];
        double y1 = p1.y / LineDetection::magnitudes[pos[i]];
        for (size_t j = i + 1; j < pos.size(); ++j) {
            const Point& p2 = LineDetection::points[pos[j]];
            double x2 = p2.x / LineDetection::magnitudes[pos[j]];
            double y2 = p2.y / LineDetection::magnitudes[pos[j]];
            double dot = (x1 * x2) + (y1 * y2);
            if (dot < bestDot) {
                bestDot = dot;
                bestI = pos[i];
                bestJ = pos[j];
            }
        }
    }

    if (bestI < 0 || bestJ < 0) {
        return false;
    }

    const Point& firstPoint = LineDetection::points[bestI];
    const Point& secondPoint = LineDetection::points[bestJ];
    pairVector = {
        (firstPoint.x / LineDetection::magnitudes[bestI]) +
            (secondPoint.x / LineDetection::magnitudes[bestJ]),
        (firstPoint.y / LineDetection::magnitudes[bestI]) +
            (secondPoint.y / LineDetection::magnitudes[bestJ])
    };

    return pointMagnitudeSquared(pairVector) > kVectorEpsilon;
}

bool resolvePcaAngle(const std::vector<int>& pos, double meanX, double meanY, double& resolvedAngle) {
    if (pos.empty()) {
        return false;
    }

    if (pos.size() == 1) {
        resolvedAngle = angleFromXY(meanX, meanY);
        return true;
    }

    double cov_xx = 0.0;
    double cov_xy = 0.0;
    double cov_yy = 0.0;
    for (int idx : pos) {
        double dx = LineDetection::points[idx].x - meanX;
        double dy = LineDetection::points[idx].y - meanY;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }

    double pointCount = static_cast<double>(pos.size());
    cov_xx /= pointCount;
    cov_xy /= pointCount;
    cov_yy /= pointCount;

    Point direction = calculatePrincipalComponent(cov_xx, cov_xy, cov_yy);
    double directionMagnitudeSquared =
        (direction.x * direction.x) + (direction.y * direction.y);
    if (directionMagnitudeSquared <= kVectorEpsilon) {
        resolvedAngle = angleFromXY(meanX, meanY);
        return true;
    }

    double projectionScale =
        -((meanX * direction.x) + (meanY * direction.y)) / directionMagnitudeSquared;
    double xIntersection = meanX + (projectionScale * direction.x);
    double yIntersection = meanY + (projectionScale * direction.y);

    if ((std::fabs(xIntersection) <= kVectorEpsilon) &&
        (std::fabs(yIntersection) <= kVectorEpsilon)) {
        resolvedAngle = angleFromXY(meanX, meanY);
    } else {
        resolvedAngle = angleFromXY(xIntersection, yIntersection);
    }
    return true;
}

}  // namespace

Point LineDetection::points[48] = {
    {86.5, 0}, {85.63, -12.219}, {80.169, -18.904}, {71.206, -24.997},
    {64.174, -35.019}, {57.142, -45.062}, {50.110, -55.104}, {43.602, -65.388},
    {44.506, -74.275}, {33.562, -79.777}, {21.954, -83.689}, {9.912, -85.394},
    {-2.326, -86.469}, {-14.518, -85.282}, {-26.423, -82.397}, {-37.805, -77.871},
    {-47.732, -71.282}, {-45.747, -61.336}, {-52.779, -51.293}, {-59.811, -41.251},
    {-66.843, -31.208}, {-73.874, -21.166}, {-84.136, -19.660}, {-86.166, -7.597},
    {-86.375, 4.650}, {-84.852, 16.804}, {-75.774, 18.931}, {-68.537, 28.788},
    {-61.506, 38.830}, {-54.474, 48.873}, {-47.442, 58.915}, {-45.728, 69.253},
    {-40.440, 76.541}, {-31.672, 72.424}, {-21.946, 65.169}, {-9.933, 63.291},
    {2.326, 63.291}, {14.583, 63.379}, {26.052, 67.343}, {34.246, 76.291},
    {44.465, 74.207}, {43.602, 65.388}, {50.110, 55.104}, {57.142, 45.062},
    {64.174, 35.019}, {71.206, 24.977}, {80.169, 18.904}, {85.633, 12.219}
};

double LineDetection::magnitudes[48] = {};

LineDetection::LineDetection()
    : angle(kNoLineAngle),
      cordLength(kNoLineAngle),
      crossLine(false),
      prevAngle(kNoLineAngle),
      previousBaseAvoidanceAngle(kNoLineAngle),
      robotHeadingDegrees(0.0),
      previousRobotHeadingDegrees(0.0),
      previousResolvedAngle(kNoLineAngle),
      hasRobotHeadingReference(false),
      hasPreviousResolvedAngle(false),
      hasPreviousBaseAvoidanceAngle(false) {
    for (int i = 0; i < 48; i++) {
        magnitudes[i] = Trig::getDist(points[i], {0, 0});
    }
    maxSensorPairDistance = 0.0;
    for (int i = 0; i < 48; ++i) {
        for (int j = i + 1; j < 48; ++j) {
            double d = Trig::getDist(points[i], points[j]);
            if (d > maxSensorPairDistance) {
                maxSensorPairDistance = d;
            }
        }
    }
    adc1.begin(cs1, mosi, miso, sck);
    adc2.begin(cs2, mosi, miso, sck);
    adc3.begin(cs3, mosi, miso, sck);
    adc4.begin(cs4, mosi, miso, sck);
    adc5.begin(cs5, mosi, miso, sck);
    adc6.begin(cs6, mosi, miso, sck);

    adcList[0] = &adc1;
    adcList[1] = &adc2;
    adcList[2] = &adc3;
    adcList[3] = &adc4;
    adcList[4] = &adc5;
    adcList[5] = &adc6;
}

void LineDetection::setRobotHeadingDegrees(double headingDegrees) {
    robotHeadingDegrees = normalize360(headingDegrees);
    if (!hasRobotHeadingReference) {
        previousRobotHeadingDegrees = robotHeadingDegrees;
    }
    hasRobotHeadingReference = true;
}

void LineDetection::clearRobotHeadingReference() {
    hasRobotHeadingReference = false;
}

void LineDetection::lineSensorDebug() {
}

void LineDetection::updateLineSensors(bool withDebug) {
    int chBig = 36;
    for (int i = 0; i < 48; i++) {
        if (i <= 12) {
            chBig = (36 + i);
        } else {
            chBig = i - 12;
        }

        sensorVals[i] = adcList[(chBig - 1) / 8]->analogRead((int)((chBig - 1) % 8));

        if (sensorVals[i] > calibrateVals[i] + kSensorTriggerOffset) {
            activatedVals[i] = 1;
        } else {
            activatedVals[i] = 0;
        }
    }

    (void)withDebug;
}

void LineDetection::Calculate() {
    updateLineSensors(false); // Make true if you want to print out all line sensor values for GUI Debug

    std::vector<int> pos;
    pos.reserve(48);
    double xTotal = 0.0;
    double yTotal = 0.0;
    int count = 0;
    int lineSensorGroups = countTriggeredGroups(activatedVals);
    for (int i = 0; i < 48; i++) {
        if (activatedVals[i] == 1) {
            pos.push_back(i);
            xTotal += points[i].x;
            yTotal += points[i].y;
            count++;
        }
    }

    bool canRotatePrevious = hasPreviousResolvedAngle && hasRobotHeadingReference;
    double rotatedPreviousAngle = previousResolvedAngle;
    if (canRotatePrevious) {
        rotatedPreviousAngle = normalize360(
            previousResolvedAngle +
            headingDeltaDegrees(previousRobotHeadingDegrees, robotHeadingDegrees));
    }

    if (count > 0) {
        xTotal /= count;
        yTotal /= count;

        Point centroid {xTotal, yTotal};
        Point origin {0, 0};
        cordLength = 1 - Trig::getDist(centroid, origin) / kIdealRingRadius;
        Serial.println("Centroid: (" + String(centroid.x) + ", " + String(centroid.y) + ")");
        Serial.println("Cord Length: " + String(cordLength));
    } else {
        cordLength = kNoLineAngle;
    }

    bool reusePreviousAngle = (count < 2) && crossLine && hasPreviousResolvedAngle;
    bool ambiguousGroupedDetection = (lineSensorGroups == 3) && hasPreviousResolvedAngle;

    double resolvedAngle = kNoLineAngle;
    bool hasResolvedAngle = false;

    if (reusePreviousAngle || ambiguousGroupedDetection) {
        resolvedAngle = canRotatePrevious ? rotatedPreviousAngle : previousResolvedAngle;
        hasResolvedAngle = true;
    } else if (count > 0) {
        double pcaAngle = kNoLineAngle;
        bool hasPcaAngle = resolvePcaAngle(pos, xTotal, yTotal, pcaAngle);

        Point oppositePairVector {0.0, 0.0};
        bool hasOppositePairVector = buildOppositePairVector(pos, oppositePairVector);
        double oppositePairAngle = hasOppositePairVector
            ? angleFromXY(oppositePairVector.x, oppositePairVector.y)
            : kNoLineAngle;

        if (hasPcaAngle) {
            resolvedAngle = pcaAngle;
            hasResolvedAngle = true;
        }

        // Preserve the reference code's opposite-pair behavior when the live
        // trigger pattern fragments into multiple islands around the ring.
        if (hasOppositePairVector && lineSensorGroups >= 2) {
            if (!hasResolvedAngle || lineSensorGroups > 2) {
                resolvedAngle = oppositePairAngle;
                hasResolvedAngle = true;
            } else if (canRotatePrevious) {
                double pairDistance = circularDistanceDegrees(oppositePairAngle, rotatedPreviousAngle);
                double pcaDistance = circularDistanceDegrees(resolvedAngle, rotatedPreviousAngle);
                if (pairDistance + 5.0 < pcaDistance) {
                    resolvedAngle = oppositePairAngle;
                }
            } else if (lineSensorGroups == 2) {
                resolvedAngle = oppositePairAngle;
            }
        }
    }

    if (hasResolvedAngle) {
        angle = normalize360(resolvedAngle);
        previousResolvedAngle = angle;
        hasPreviousResolvedAngle = true;

        // Track the "away from line" direction independently from crossLine.
        // This mirrors the older line-cross logic more closely: when that base
        // direction flips by more than 90 degrees, we have likely crossed the
        // boundary and should drive back over it.
        bool hasReliableBaseDirection = (count >= 2) && !reusePreviousAngle && !ambiguousGroupedDetection;
        double baseAvoidanceAngle = normalize360(angle + 180.0);
        if (hasReliableBaseDirection) {
            if (hasPreviousBaseAvoidanceAngle &&
                circularDistanceDegrees(baseAvoidanceAngle, previousBaseAvoidanceAngle) >
                    kCrossToggleThresholdDegrees) {
                crossLine = !crossLine;
            }
            previousBaseAvoidanceAngle = baseAvoidanceAngle;
            hasPreviousBaseAvoidanceAngle = true;
        }
    } else {
        crossLine = false;
        angle = kNoLineAngle;
        prevAngle = angle;
        cordLength = kNoLineAngle;
        hasPreviousResolvedAngle = false;
        previousBaseAvoidanceAngle = kNoLineAngle;
        hasPreviousBaseAvoidanceAngle = false;
    }

    if (hasRobotHeadingReference) {
        previousRobotHeadingDegrees = robotHeadingDegrees;
    }
}

double LineDetection::getAngle() {
    return angle;
}

double LineDetection::avoidanceAngle() {
    if (angle == kNoLineAngle) {
        return kNoLineAngle;
    }

    prevAngle = angle;

    if (crossLine) {
        return angle;
    }

    return normalize360(angle + 180.0);
}

bool LineDetection::getCrossLine() const {
    return crossLine;
}

double LineDetection::getCordLength() {
    return cordLength;
}

double LineDetection::getChordLengthFurthestPairNormalized() {
    int idx[48];
    int k = 0;
    for (int i = 0; i < 48; ++i) {
        if (activatedVals[i] == 1) {
            idx[k++] = i;
        }
    }
    if (k < 2) {
        return -5.0;
    }

    double maxD2 = 0.0;
    for (int i = 0; i < k; ++i) {
        const Point& pi = points[idx[i]];
        for (int j = i + 1; j < k; ++j) {
            const Point& pj = points[idx[j]];
            double dx = pi.x - pj.x;
            double dy = pi.y - pj.y;
            double d2 = (dx * dx) + (dy * dy);
            if (d2 > maxD2) {
                maxD2 = d2;
            }
        }
    }

    double furthest = std::sqrt(maxD2);
    double n = furthest / maxSensorPairDistance;
    if (n > 1.0) {
        n = 1.0;
    }
    return n;
}
