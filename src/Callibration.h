#ifndef Calibration_H
#define Calibration_H

// Forward declarations instead of includes
class LineDetection;  // Forward declaration
class CompassSensor;  // Forward declaration

class Calibration {
    public:
        Calibration(LineDetection& lineDetection, CompassSensor& compassSensor);
        void calibrateLineSensors(); 
        void calibrateCompassSensor();
        int calibrateVal[24];
    private:
        LineDetection& lineDetection;
        CompassSensor& compassSensor;
};

#endif // Calibration_H
