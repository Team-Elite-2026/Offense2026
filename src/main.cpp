#include <Arduino.h>
#include <LineDetection.h>
#include <CompassSensor.h>
#include <Switches.h>
#include <Callibration.h>

LineDetection lineDetection;
CompassSensor compassSensor;
Switch switches;
Calibration calibration(lineDetection,  compassSensor);


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  compassSensor.begin();
    // calibration.calibrateCompassSensor();
}



void loop() {
  if (switches.calibration()) {
    calibration.calibrateLineSensors();
    Serial.println("Calibrating");
  } else {
  // Serial.println("Testing Line Sensors");
    Serial.println("Line angle: " + String(lineDetection.getLineAngle()));
  }
  // put your main code here, to run repeatedly:
}
