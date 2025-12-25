#include <Callibration.h>
#include <math.h>
#include <LineDetection.h>
#include <CompassSensor.h>
#include <Movement.h>

Calibration::Calibration(LineDetection& lineDetection, CompassSensor& compassSensor)
    : lineDetection(lineDetection), compassSensor(compassSensor) {
}


void Calibration::calibrateLineSensors() {
    lineDetection.updateLineSensors(false);
    for (int i = 0; i < 48; i++) {
        lineDetection.calibrateVals[i] = fmax(lineDetection.calibrateVals[i],lineDetection.sensorVals[i] * 1.5);
    }
}

void Calibration::calibrateCompassSensor() {
    compassSensor.zeroedAngle = compassSensor.getOrientation();
    Serial.println("Zeroed Angle: " + String(compassSensor.zeroedAngle));
    Serial.println();
}