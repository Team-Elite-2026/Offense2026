#include <VirtualBoundary.h>

#include <math.h>
#include <Arduino.h>

VirtualBoundary::VirtualBoundary(const VirtualBoundaryBounds& bounds)
    : _bounds(bounds) {
}

double VirtualBoundary::normalize360(double angle)
{
    while (angle < 0.0)
    {
        angle += 360.0;
    }
    while (angle >= 360.0)
    {
        angle -= 360.0;
    }
    return angle;
}

bool VirtualBoundary::contains(const Point& currentPose) const
{
    return currentPose.x >= _bounds.minX &&
           currentPose.x <= _bounds.maxX &&
           currentPose.y >= _bounds.minY &&
           currentPose.y <= _bounds.maxY;
}

bool VirtualBoundary::getAvoidanceAngle(const Point& currentPose,
                                        double robotHeadingDegrees,
                                        double& robotRelativeAvoidanceAngle) const
{
    double fieldX = 0.0;
    double fieldY = 0.0;

    if (currentPose.x < _bounds.minX)
    {
        fieldX += 1.0;
    }
    else if (currentPose.x > _bounds.maxX)
    {
        fieldX -= 1.0;
    }

    if (currentPose.y < _bounds.minY)
    {
        fieldY += 1.0;
    }
    else if (currentPose.y > _bounds.maxY)
    {
        fieldY -= 1.0;
    }

    Serial.print("Field X: ");
    Serial.println(fieldX);
    Serial.print("Field Y: ");
    Serial.println(fieldY);

    if ((fieldX * fieldX + fieldY * fieldY) < 1e-6)
    {
        return false;
    }
    Serial.println("Field Avoidance Angle: "); 
    Serial.println(Trig::toDegrees(atan2(fieldX, fieldY)));
    double fieldAvoidanceAngle = Trig::toDegrees(atan2(fieldX, fieldY));
    robotRelativeAvoidanceAngle = normalize360(fieldAvoidanceAngle - robotHeadingDegrees);
    return true;
}
