#include <Arduino.h>
#include <LineDetection.h>
#include <CompassSensor.h>
#include <Switches.h>
#include <Callibration.h>
#include <Movement.h>


double pincontrolRLA = 22;
double pincontrolRLB = 23;
double pinspeedRL = 2;
double pincontrolRRA = 20;
double pincontrolRRB = 21;
double pinspeedRR = 3;
double pincontrolFRA = 18;
double pincontrolFRB = 19;
double pinspeedFR = 4;
double pincontrolFLA = 9;
double pincontrolFLB = 10;
double pinspeedFL = 5;

LineDetection lineDetection;
CompassSensor compassSensor;
Switch switches;
Calibration calibration(lineDetection,  compassSensor);
Motor FL(pincontrolFLA, pincontrolFLB, pinspeedFL);
Motor FR(pincontrolFRA, pincontrolFRB, pinspeedFR);
Motor BL(pincontrolRLA, pincontrolRLB, pinspeedRL);
Motor BR(pincontrolRRA, pincontrolRRB, pinspeedRR);
Movement movement(FL, FR, BL, BR, compassSensor);

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  compassSensor.begin();
  compassSensor.callibrate();
}

void testingCompass() {
    movement.movement(0,0.15,0, true);
    Serial.println(compassSensor.currentOffset());
}

void loop() {
  if (switches.calibration()) {
    calibration.calibrateLineSensors();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
  } else if (!switches.start()) {
    movement.stop();
  }
  else {
  // Serial.println("Testing Line Sensors");
    lineDetection.Calculate();
    double angle = lineDetection.getAngle();
    Serial.println("Line Angle: " + String(angle));
    if (angle == -5) {
      movement.stop();
    } else {
      double avoidanceAngle = lineDetection.avoidanceAngle();
      Serial.println("Avoidance angle: " + String(avoidanceAngle));
      movement.movement(avoidanceAngle,0.15,0, true);
    }

  }
}
