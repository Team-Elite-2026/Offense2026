#include <trig.h>
#include <math.h>

double Trig::toRadians(double degrees) {
    return degrees * (M_PI / 180);
}

double Trig::toDegrees(double radians) {
    return radians * (180 / M_PI);
}

double Trig::Cos(double degrees) {
    return cos(toRadians(degrees));
}

double Trig::Sin(double degrees) {
    return sin(toRadians(degrees));
}

double Trig::avg(double a, double b) {
    return (a + b) / 2;
}

double Trig::getSlope(Point p1, Point p2) {
    return (p2.y - p1.y) / (p2.x - p1.x);
}

double Trig::getDist(Point p1, Point p2) {
    return sqrt(pow((p1.y-p2.y),2) + pow((p1.x-p2.x),2));
}

double Trig::wrapAngle(double angle) {
    if (angle < -180) angle += 360;
    if (angle >   180) angle -= 360;
    return angle;
}