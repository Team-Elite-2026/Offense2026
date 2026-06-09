#include <Arduino.h>
#include <CompassSensor.h>
#include <Switches.h>
#include <Callibration.h>
#include <Movement.h>
#include <orbit.h>
#include <Cam.h>
#include <Defense.h>
#include <trig.h>
#include <string.h>
#include <TrajectoryExecutor.h>
#include <LcdController.h>
#include <LinePCBComm.h>

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

CompassSensor compassSensor;
Switch switches;
Calibration calibration(compassSensor);
Motor FL(pincontrolFLA, pincontrolFLB, pinspeedFL);
Motor FR(pincontrolFRA, pincontrolFRB, pinspeedFR);
Motor BL(pincontrolRLA, pincontrolRLB, pinspeedRL);
Motor BR(pincontrolRRA, pincontrolRRB, pinspeedRR);
Movement movement(FL, FR, BL, BR, compassSensor);
Orbit orbit(1);
Cam camera;
Defense defense;
LinePCBComm linePCBComm(Serial2);  // Serial2: LinePCB Teensy 4.0 link (1 Mbaud)
TrajectoryExecutor trajectoryExecutor(FL, FR, BL, BR, compassSensor, switches);

RobotMode kRobotMode = RobotMode::Offense;
LcdController lcdController(Serial8, linePCBComm, compassSensor, switches, movement, kRobotMode);

double defenseSpeedFactor = 0.26;
double offenseSpeedFactor = 0.22;
double lineAvoidanceSpeed = 0.15;
double lineAngle, currentOffset, orbitAngle, maxChordLength, goalAngle, avoidanceAngle;
bool aimingGoal;

double getHomeGoalAngle()
{
  if (lcdController.isGoalBlueSelected())
  {
    return camera.yellowGoal;
  }
  return camera.blueGoal;
}

bool runRequestedCalibration()
{
  if (switches.calibration())
  {
    movement.stop();
    linePCBComm.triggerCalibration();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    lcdController.sendCalibrationStatus();
    return true;
  }

  if (lcdController.state.lineCalibrationActive)
  {
    movement.stop();
    linePCBComm.triggerCalibration();
    lcdController.sendCalibrationStatus();
    return true;
  }

  return false;
}

void setup()
{
  lcdController.applyRobotModeSettings();
  Serial.begin(9600);
  lcdController.begin(9600);
  Serial3.begin(2000000);   // Pi <-> Teensy 4.1 (was Serial2)
  linePCBComm.begin(1000000);  // LinePCB Teensy 4.0 link
  compassSensor.begin();
  compassSensor.callibrate();
}

void runOffense()
{
  avoidanceAngle = -5;
  if (runRequestedCalibration())
  {
    movement.stop();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    return;
  }

  // Serial.println("Testing Line Sensors");
  camera.CamCalc();
  lineAngle = linePCBComm.getLineAngle();
  orbitAngle = orbit.CalculateRobotAngle(camera.ballAngle, camera.ballDist);
  if (lcdController.isGoalBlueSelected())
  {
    Serial.println("blue goal");
    goalAngle = camera.blueGoal;
  }
  else
  {
    Serial.println("yellow goal");
    goalAngle = camera.yellowGoal;
  }

  if (goalAngle == -5)
  {
    goalAngle  = 0;
    aimingGoal = false;
  }
  else
  {
    aimingGoal = true;
  }

  Serial.println("Line Angle: "        + String(lineAngle));
  Serial.println("Robot Angle: "       + String(orbitAngle));
  Serial.println("Ball Angle: "        + String(camera.ballAngle));
  Serial.println("Goal Angle: "        + String(goalAngle));
  Serial.println("Ball dist: "         + String(camera.ballDist));
  Serial.println("Zeroed angle: "      + String(compassSensor.currentOffset()));
  Serial.println("Orientation angle: " + String(compassSensor.getOrientation()));

  movement.kickBackground();

  // Always drain Serial3 so no trajectory packets are silently dropped,
  // even when line avoidance overrides movement this iteration.
  trajectoryExecutor.processSerial();

  if (!switches.start())
  {
    movement.stop();
    return;
  }

  // Line avoidance — highest-priority safety override
  if (lineAngle != -5)
  {
    avoidanceAngle = linePCBComm.getAvoidanceAngle();
    Serial.println("Avoidance angle: " + String(avoidanceAngle));
    movement.movement(avoidanceAngle, lineAvoidanceSpeed, 0, false);
    return;
  }

  // Kicker: fire when ball is secure and we are aligned with the goal
  // THIS WILL HAVE TO CHANGE
  if (switches.lightgate() && fabs(goalAngle) < 5)
  {
    movement.kick();
    return;
  }

  // Trajectory execution (Pipeline.md Steps 7-8)
  if (!trajectoryExecutor.execute())
  {
    if (camera.ballAngle != -5)
    {
      movement.movement(orbitAngle, offenseSpeedFactor, goalAngle, aimingGoal);
    }
    else
    {
      movement.stop();
    }
  }
}

void runDefense()
{
  avoidanceAngle = -5;
  if (runRequestedCalibration())
  {
    return;
  }

  lineAngle = linePCBComm.getLineAngle();
  maxChordLength = linePCBComm.getChordLength();
  if (lineAngle != -5)
  {
    avoidanceAngle = linePCBComm.getAvoidanceAngle();
    Serial.println("Avoidance Angle: " + String(avoidanceAngle));
  }
  bool crossLineState = linePCBComm.getCrossLine();

  double homeGoalAngle = getHomeGoalAngle();
  movement.kickBackground();

  currentOffset = compassSensor.currentOffset();

  Serial.println("BALL DISTANCE: " + String(camera.ballDist));

  lcdController.sendTelemetry(lineAngle, avoidanceAngle);

  if (!lcdController.isStartEnabled())
  {
    movement.stop();
    return;
  }

  if (camera.ballAngle == -5)
  {
    movement.stop();
    return;
  }

  if (homeGoalAngle == -5)
  {
    movement.movement(camera.ballAngle, defenseSpeedFactor, 0, false);
    return;
  }
}

void loop()
{
  linePCBComm.update();
  linePCBComm.setRobotHeadingDegrees((float)compassSensor.getOrientation());
  trajectoryExecutor.setMouseVelocity(linePCBComm.getMouseVx(), linePCBComm.getMouseVy());

  lcdController.readCommands();
  if (kRobotMode == RobotMode::Offense)
  {
    runOffense();
  }
  else
  {
    runDefense();
  }
}
