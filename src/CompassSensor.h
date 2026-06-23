#ifndef COMPASSSENSOR_H
#define COMPASSSENSOR_H

#include <Wire.h>
#include <Adafruit_BNO055.h>

class CompassSensor {
    public:
    CompassSensor();
    void callibrate(Print* statusOutput = nullptr);
    void begin();
    // Read heading and omega from BNO055 once per loop; getOrientation() and
    // getOmegaRadS() return the cached values without issuing I²C transactions.
    void sample();
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
    sensors_event_t gyroEvent_;

};
#endif // COMPASSSENSOR_H
