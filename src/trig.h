#ifndef trig_h
#define trig_h

struct Point {
    double x;
    double y;
};

class Trig {
public:
    static double toRadians(double degrees);
    static double Sin(double angle);
    static double Cos(double angle);
    static double avg(double a, double b);
    static double toDegrees(double radians);
    static double getSlope(Point p1, Point p2);
    static double getDist(Point p1, Point p2);
    static double wrapAngle(double angle);
};

#endif
