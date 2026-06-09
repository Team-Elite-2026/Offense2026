#ifndef Calibration_H
#define Calibration_H

class CompassSensor;  // Forward declaration

// Line sensor calibration is handled by LinePCBController on the LinePCB Teensy.
// Use LinePCBComm::triggerCalibration() to trigger a line-sensor calibration pass.
class Calibration {
    public:
        explicit Calibration(CompassSensor& compassSensor);
        void calibrateCompassSensor();
    private:
        CompassSensor& compassSensor;
};

#endif // Calibration_H
