#ifndef trig_h
#define trig_h

struct Point {
    double x;
    double y;
    double heading = 0;
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
    static double dotProduct(int num1, int num2);
    static double clamp(double value, double minValue, double maxValue);
    static double normalize360(double angle);
    static double normalize180(double angle);
    static double angularDistance(double a, double b);
    static double angleFromVector(double x, double y);
    static double wrapAngle(double angle);
    static double min(double a, double b);
    static double getAngle(Point p1, Point p2); // Returns the angle between the vertical of p1 and p2
};

#endif
