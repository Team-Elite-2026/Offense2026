#ifndef COMPASSSENSOR_H
#define COMPASSSENSOR_H

#include <Wire.h>
#include <Adafruit_BNO055.h>

class CompassSensor {
    public:
    CompassSensor();
    void callibrate();
    void begin();
    int currentOffset();
    int getOrientation();
    float getOmegaRadS();
    int currentOffset(double goalAngle);
    int currentFieldRelativeOffset(double goalAngle);
    double robotRelativeToField(double robotRelativeAngle);
    double zeroedAngle;

    private:
    Adafruit_BNO055 bno;
    sensors_event_t event;
    
};
#endif // COMPASSSENSOR_H
