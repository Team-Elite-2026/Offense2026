#include <Arduino.h>
#include <CompassSensor.h>
#include <Switches.h>
#include <Callibration.h>
#include <Movement.h>
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
Defense defense;
LinePCBComm linePCBComm(Serial2);  // Serial2: LinePCB Teensy 4.0 link (1 Mbaud)
TrajectoryExecutor trajectoryExecutor(FL, FR, BL, BR, compassSensor, switches, movement);

RobotMode kRobotMode = RobotMode::Offense;
LcdController lcdController(Serial8, linePCBComm, compassSensor, switches, movement, kRobotMode);

double lineAvoidanceSpeed = 0.15;
double lineAngle, avoidanceAngle;

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

// The Pi owns ALL motion planning for both roles and streams ready-to-execute
// action chunks. The role (offense/defense) is decided entirely on the Pi from
// the LCD selection relayed in telemetry, so the Teensy is role-agnostic: it
// executes whatever chunks arrive, with line avoidance as the top-priority
// safety override. Switching a robot's role mid-match is therefore just the Pi
// changing which chunks it sends — no Teensy-side change.
void runRobot()
{
  avoidanceAngle = -5;
  if (runRequestedCalibration())
  {
    movement.stop();
    return;
  }

  lineAngle = linePCBComm.getLineAngle();
  if (lineAngle != -5)
  {
    avoidanceAngle = linePCBComm.getAvoidanceAngle();
  }

  movement.kickBackground();

  // Single owner of the Pi link (Serial3): drain it and run the chunk/pong
  // framing state machine every loop so no packets are dropped.
  trajectoryExecutor.processSerial();

  lcdController.sendTelemetry(lineAngle, avoidanceAngle);

  if (!lcdController.isStartEnabled())
  {
    movement.stop();
    return;
  }

  // Line avoidance — highest-priority safety override.
  if (lineAngle != -5)
  {
    movement.movement(avoidanceAngle, lineAvoidanceSpeed, 0, false);
    return;
  }

  // Execute the Pi's planned trajectory (Pipeline.md Steps 7-8). The executor's
  // own freshness logic (clock sync + 20 ms grace + emergency brake) decides
  // when a chunk is still valid; with no live chunk, hold position.
  if (!trajectoryExecutor.execute())
  {
    movement.stop();
  }
}

void loop()
{
  linePCBComm.update();
  linePCBComm.setRobotHeadingDegrees((float)compassSensor.getOrientation());
  trajectoryExecutor.setMouseVelocity(linePCBComm.getMouseVx(), linePCBComm.getMouseVy());

  lcdController.readCommands();
  trajectoryExecutor.setMatchState(lcdController.isStartEnabled(),
                                   lcdController.isGoalBlueSelected(),
                                   lcdController.telemetryModeOverride());

  // Role is Pi-driven; the Teensy executes chunks for whatever role the Pi sends.
  runRobot();
}
