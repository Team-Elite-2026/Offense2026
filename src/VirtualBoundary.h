#ifndef VIRTUAL_BOUNDARY_H
#define VIRTUAL_BOUNDARY_H

#include <trig.h>

struct VirtualBoundaryBounds {
    double minX;
    double maxX;
    double minY;
    double maxY;
};

class VirtualBoundary {
public:
    explicit VirtualBoundary(const VirtualBoundaryBounds& bounds);

    bool getAvoidanceAngle(const Point& currentPose,
                           double robotHeadingDegrees,
                           double& robotRelativeAvoidanceAngle) const;

    bool contains(const Point& currentPose) const;

private:
    VirtualBoundaryBounds _bounds;
};

#endif
