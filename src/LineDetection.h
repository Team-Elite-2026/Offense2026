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
        void Calculate();
        static Point points[48];
        void updateLineSensors(bool withDebug);
        void lineSensorDebug();
        void setRobotHeadingDegrees(double headingDegrees);
        void clearRobotHeadingReference();
        double sensorVals[48];
        static double magnitudes[48];
        double calibrateVals[48];
        int activatedVals[48];
        double getAngle();
        double avoidanceAngle();
        bool getCrossLine() const;
        double getCordLength();
        double getChordLengthFurthestPairNormalized(); // largest distance between two activated sensors [0,1]

    private:
        double angle;
        double cordLength;
        /** Precomputed max distance between any two points[i], points[j] on the ring layout. */
        double maxSensorPairDistance;
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

        bool crossLine;
        double prevAngle;
        double previousBaseAvoidanceAngle;
        double robotHeadingDegrees;
        double previousRobotHeadingDegrees;
        double previousResolvedAngle;
        bool hasRobotHeadingReference;
        bool hasPreviousResolvedAngle;
        bool hasPreviousBaseAvoidanceAngle;
};

#endif 
