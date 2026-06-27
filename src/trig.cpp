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

// double Trig::dotProduct(int sensNum1, int sensNum2) {
//     Point p1 = LineDetection::points[sensNum1];
//     Point p2 = LineDetection::points[sensNum1];
//     double x1 = p1.x / LineDetection::magnitudes[sensNum1];
//     double y1 = p1.y / LineDetection::magnitudes[sensNum1];
//     double x2 = p2.x / LineDetection::magnitudes[sensNum2];
//     double y2 = p2.y / LineDetection::magnitudes[sensNum2];
//     return (x1 * x2) + (y1* y2);
// }

double Trig::wrapAngle(double angle) {
    if (angle < -180) angle += 360;
    if (angle >   180) angle -= 360;
    return angle;
}


double Trig::min(double a, double b) {
    return (a < b) ? a : b;

}

double Trig::getAngle(Point p1, Point p2) {
    // Field frame (zeroed/centered): +y points toward the attacked goal (the
    // direction the robot faces at zero heading) and +x is the robot's right at
    // zero heading.
    //
    // movement() steers RELATIVE TO THE ROBOT'S BODY (0 = drive out the robot's
    // front), not relative to the zeroed direction -- the wheels are fixed to the
    // chassis and the heading correction only adds a rotation, never rotates the
    // translation vector. So movement(a) drives toward field bearing (heading + a).
    //
    // The field bearing to the target, clockwise from +y, is atan2(dx, dy). To
    // turn that into the body-relative command movement() needs, we subtract the
    // robot's current heading (p1.heading, also clockwise from +y since heading 0
    // faces the attacked goal). This stays correct at ANY heading -- including
    // while PlanToPose is holding the robot at the shot pose's 180-degree heading.
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double angle = toDegrees(atan2(dx, dy)) - p1.heading;

    while (angle < 0) {
        angle += 360;
    }
    while (angle >= 360) {
        angle -= 360;
    }

    return angle;
}
