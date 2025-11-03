#ifndef LINEDETECTION_H
#define LINEDETECTION_H


struct Point {
    double x;
    double y;
};

class LineDetection {
    public:
        LineDetection();
        double getLineAngle(int sensNum1, int sensNum2);
        Point points[48];
};

#endif 