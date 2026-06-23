#include <Arduino.h>
#include <CompassSensor.h>
#include <Callibration.h>
#include <Movement.h>
#include <Defense.h>
#include <trig.h>
#include <string.h>
#include <TrajectoryExecutor.h>
#include <ModeControl.h>
#include <LinePCBComm.h>

double pincontrolRLA = 23;
double pincontrolRLB = 22;
double pinspeedRL = 2;
double pincontrolRRA = 31;
double pincontrolRRB = 18;
double pinspeedRR = 4;
double pincontrolFRA = 20;
double pincontrolFRB = 21;
double pinspeedFR = 3;
double pincontrolFLA = 9;
double pincontrolFLB = 10;
double pinspeedFL = 5;

CompassSensor compassSensor;
Calibration calibration(compassSensor);
double pincontrolDribblerA = 11;
double pincontrolDribblerB = 12;
double pinspeedDribbler = 6;

Motor FL(pincontrolFLA, pincontrolFLB, pinspeedFL);
Motor FR(pincontrolFRA, pincontrolFRB, pinspeedFR);
Motor BL(pincontrolRLA, pincontrolRLB, pinspeedRL);
Motor BR(pincontrolRRA, pincontrolRRB, pinspeedRR);
Motor Dribbler(pincontrolDribblerA, pincontrolDribblerB, pinspeedDribbler);
Movement movement(FL, FR, BL, BR, compassSensor);
Defense defense;
LinePCBComm linePCBComm(Serial2);  // Serial2: LinePCB Teensy 4.0 link (1 Mbaud)
TrajectoryExecutor trajectoryExecutor(FL, FR, BL, BR, compassSensor, movement);

RobotMode kRobotMode = RobotMode::Offense;
ModeControl modeControl(Serial8, linePCBComm, compassSensor, movement, kRobotMode);

double lineAvoidanceSpeed = 0.15;
double lineAngle, avoidanceAngle;

bool runRequestedCalibration()
{
  // if (modeControl.state.lineCalibrationActive)
  // {
  //   movement.stop();
  //   linePCBComm.triggerCalibration();
  //   modeControl.sendCalibrationStatus();
  //   return true;
  // }
  return false;
}

static uint8_t serial3RxBuf[4096];

void setup()
{
  Serial.begin(9600);
  modeControl.begin(9600);
  modeControl.applyRobotModeSettings();
  Serial3.begin(2000000);   // Pi <-> Teensy 4.1 (was Serial2)
  Serial3.addMemoryForRead(serial3RxBuf, sizeof(serial3RxBuf));
  // linePCBComm.begin(1000000);  // LinePCB Teensy 4.0 link
  compassSensor.begin();
  compassSensor.callibrate();
  Serial.println("compass callibration is done");
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
  // if (runRequestedCalibration())
  // {
  //   movement.stop();
  //   return;
  // }

  lineAngle = linePCBComm.getLineAngle();
  if (lineAngle != -5)
  {
    avoidanceAngle = linePCBComm.getAvoidanceAngle();
  }

  movement.kickBackground();

  // Single owner of the Pi link (Serial3): drain it and run the chunk/pong
  // framing state machine every loop so no packets are dropped.
  trajectoryExecutor.processSerial();

  // modeControl.sendTelemetry(lineAngle, avoidanceAngle);

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
  compassSensor.sample();  // single I²C burst for heading + omega; all callers use cache
  // linePCBComm.update();
  // linePCBComm.setRobotHeadingDegrees((float)compassSensor.getOrientation());
  // trajectoryExecutor.setMouseVelocity(linePCBComm.getMouseVx(), linePCBComm.getMouseVy());

  // While the rocker switch is off, continuously track current heading as zero.
  // The moment it is flipped on, the last-seen orientation becomes the field zero.
  if (!modeControl.isStartEnabled())
  {
    compassSensor.zeroedAngle = compassSensor.getOrientation();
    movement.stop();
  }
  else {
    runRobot();
  }

  // movement.testMotorsTogether();
  // movement.stop();
 //  movement.movement(90,0.2,0,false);

  // modeControl.readCommands();
  // trajectoryExecutor.setMatchState(modeControl.isStartEnabled(),
  //                                  modeControl.isGoalBlueSelected(),
  //                                  modeControl.telemetryModeOverride());

  // Role is Pi-driven; the Teensy executes chunks for whatever role the Pi sends.
}
