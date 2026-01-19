#include <Arduino.h>
#include <LineDetection.h>
#include <CompassSensor.h>
#include <Switches.h>
#include <Callibration.h>
#include <Movement.h>
#include <orbit.h>
#include <Cam.h>

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
Orbit orbit(1);
Cam camera;

double lineAngle;
double robotAngle;
double goalAngle;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  Serial2.begin(2000000);
  compassSensor.begin();
  compassSensor.callibrate();
}

void testingCompass() {
    movement.movement(0,0.15,0);
    Serial.println(compassSensor.currentOffset());
}

void loop() {
  if (switches.calibration()) {
    calibration.calibrateLineSensors();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
  }
  else {
  // Serial.println("Testing Line Sensors");
    lineDetection.Calculate();
    camera.CamCalc();
    lineAngle = lineDetection.getAngle();
    robotAngle = orbit.CalculateRobotAngle(camera.ballAngle, camera.ballDist);
    if (switches.goalSide()) {
      Serial.println("blue goal");
      goalAngle = camera.blueGoal;
    }
    else {
      Serial.println("yellow goal");
      goalAngle = camera.yellowGoal;
    }
    Serial.println("Line Angle: " + String(lineAngle));
    Serial.println("Robot Angle: " + String(robotAngle));
    Serial.println("Ball Angle: " + String(camera.ballAngle));
    Serial.println("Goal Angle: " + String(goalAngle));
    movement.kickBackground();
    if (lineAngle == -5) {
      if (switches.start()){
        if(switches.lightgate()) {
          movement.movement(0,0.2,goalAngle);
          movement.kick();
        }
        else if(camera.ballAngle != -5)
          movement.movement(robotAngle,0.2,goalAngle);
        else
          movement.stop();
      }
      else
        movement.stop();
    } else {
      double avoidanceAngle = lineDetection.avoidanceAngle();
      Serial.println("Avoidance angle: " + String(avoidanceAngle));
      if (switches.start())
        movement.movement(avoidanceAngle,0.2,goalAngle);
      else
        movement.stop();
    }

  }
}
