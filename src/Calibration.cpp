#include <Callibration.h>
#include <CompassSensor.h>

Calibration::Calibration(CompassSensor& compassSensor)
    : compassSensor(compassSensor) {
}

void Calibration::calibrateCompassSensor() {
    compassSensor.zeroedAngle = compassSensor.getOrientation();
    Serial.println("Zeroed Angle: " + String(compassSensor.zeroedAngle));
    Serial.println();
}