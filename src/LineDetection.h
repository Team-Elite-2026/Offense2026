#ifndef LINEDETECTION_H
#define LINEDETECTION_H

#include <MCP3XXX.h>



struct Point {
    double x;
    double y;
};

class LineDetection {
    public:
        LineDetection();
        double getLineAngle();
        Point points[48];
        void updateLineSensors();
        void lineSensorDebug();
        double sensorVals[48];
        double calibrateVals[48];

    private:
        MCP3008 adc1;
        MCP3008 adc2;
        MCP3008 adc3;
        MCP3008 adc4;
        MCP3008 adc5;
        MCP3008 adc6;

        int cs1 = 14;
        int cs2 = 15;
        int cs3 = 16;
        int cs4 = 37;
        int cs5 = 36;
        int cs6 = 35;
        int sck = 13;
        int mosi = 11;
        int miso = 12;

        MCP3008* adcList[6];
};

#endif 