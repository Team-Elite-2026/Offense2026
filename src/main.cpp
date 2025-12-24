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

void lineSensorTesting() {
  // lineDetection.lineSensorDebug();
  Serial.println("Line angle: " + String(lineDetection.getLineAngle()));
}

void loop() {
  if (switches.calibration()) {
    calibration.calibrateLineSensors();
    Serial.println("Calibratign");
  } else {
  Serial.println("Testing Line Sensors");
  lineSensorTesting();
  }
  // put your main code here, to run repeatedly:
}
