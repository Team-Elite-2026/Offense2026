#include <Arduino.h>
#include <math.h>

#include <Cam.h>
#include <Callibration.h>
#include <CompassSensor.h>
#include <LinePCBComm.h>
#include <ModeControl.h>
#include <Movement.h>
#include <OffenseStateMachine.h>
#include <RobotConfig.h>
#include <Switches.h>
#include <orbit.h>
#include <Defense.h>

// Configure the active offense mode here while the automatic transitions are
// still being developed.
constexpr OffenseState kConfiguredOffenseState = OffenseState::Orbit;

constexpr unsigned long kPiHeadingTelemetryIntervalMs = 100;

RobotMode kRobotMode = defaultRobotMode;

CompassSensor compassSensor;
Switch switches;
Calibration calibration(compassSensor);
Motor* FL = nullptr;
Motor* FR = nullptr;
Motor* BL = nullptr;
Motor* BR = nullptr;
Motor* dribbler = nullptr;
Movement* movement = nullptr;
Defense defense;
Orbit orbit(1);
Cam camera;
LinePCBComm linePCBComm(Serial2);
ModeControl* modeControl = nullptr;
OffenseStateMachine* offenseStateMachine = nullptr;
unsigned long lastPiHeadingTelemetryMs = 0;
double lineAngle, currentOffset, orbitAngle, maxChordLength, goalAngle, avoidanceAngle;
static void initializeDriveMotors()
{
  pinMode(selectionPin, INPUT);
  const bool useDefaultMotorLayout = digitalRead(selectionPin) == LOW;

  if (useDefaultMotorLayout)
  {
    // Robot 2
    FL = new Motor(pincontrolFLA, pincontrolFLB, pinspeedFL);
    FR = new Motor(pincontrolFRA, pincontrolFRB, pinspeedFR);
    BL = new Motor(pincontrolRLA, pincontrolRLB, pinspeedRL);
    BR = new Motor(pincontrolRRA, pincontrolRRB, pinspeedRR);
  }
  else
  {
    //Robot 1
    FL = new Motor(pincontrolFLA, pincontrolFLB, pinspeedFL);
    FR = new Motor(pincontrolFRB, pincontrolFRA, pinspeedFR);
    BL = new Motor(pincontrolRLB, pincontrolRLA, pinspeedRL);
    BR = new Motor(pincontrolRRB, pincontrolRRA, pinspeedRR);
  }

  dribbler = new Motor(pincontrolDribblerA, pincontrolDribblerB, pinspeedDribbler);
}

static void sendHeadingTelemetryToPi()
{
  const unsigned long now = millis();
  if (now - lastPiHeadingTelemetryMs < kPiHeadingTelemetryIntervalMs)
  {
    return;
  }
  lastPiHeadingTelemetryMs = now;

  Serial3.print("T,heading=");
  Serial3.print(compassSensor.currentOffset());
  Serial3.println();
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Testing Run");
  Serial3.begin(2000000);

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(6, OUTPUT);

  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(2, OUTPUT);

  pinMode(21, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(3, OUTPUT);

  pinMode(18, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(4, OUTPUT);

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(5, OUTPUT);

  pinMode(30, OUTPUT);
  pinMode(selectionPin, INPUT);

  initializeDriveMotors();
  movement = new Movement(*FL, *FR, *BL, *BR, *dribbler, compassSensor);
  camera.setMovement(movement);
  modeControl = new ModeControl(Serial8, linePCBComm, compassSensor, *movement, kRobotMode);
  modeControl->begin(115200);
  modeControl->sendBootMarker();

  compassSensor.begin();
  compassSensor.callibrate(modeControl->statusOutput());
  calibration.calibrateCompassSensor();
  linePCBComm.begin(1000000);

  offenseStateMachine = new OffenseStateMachine(
    compassSensor,
    calibration,
    linePCBComm,
    camera,
    orbit,
    *movement,
    *modeControl);
}

int getHomeGoalAngle() {
  if (modeControl->isGoalBlueSelected()) {
    return camera.yellowGoal;
  } 
  return camera.blueGoal;
}

void runDefense()
{
  offenseStateMachine->updateVisionAndLineState();
  
  lineAngle = linePCBComm.getLineAngle();
  maxChordLength = linePCBComm.getChordLength();
  if (lineAngle != -5)
  {
    // Updates crossLine side memory based on angle wrap jumps.
    avoidanceAngle = linePCBComm.getAvoidanceAngle();
    Serial.println("Avoidance Angle: " + String(avoidanceAngle));
  }
  bool crossLineState = linePCBComm.getCrossLine();

  double homeGoalAngle = getHomeGoalAngle();
  movement->kickBackground();

  currentOffset = compassSensor.currentOffset();

  Serial.println("Line Angle: " + String(lineAngle));
  Serial.println("Ball Angle: " + String(camera.ballAngle));
  Serial.println("Home Goal Angle: " + String(homeGoalAngle));
  Serial.println("Max Normalized Activated Sensor Distance: " + String(maxChordLength));
  Serial.println("Cross Line: " + String(crossLineState ? "true" : "false"));
  Serial.println("Current offset: " + String(currentOffset));
  Serial.println("Ball Angle: " + String(camera.ballAngle));

  if (!modeControl->isStartEnabled())
  {
    movement->stop();
    return;
  }

  if (camera.ballAngle == -5)
  {
    movement->stop();
    return;
  }

  if (homeGoalAngle == -5)
  {
    movement->movement(camera.ballAngle, defenseSpeedFactor, 0, false);
    return;
  }

  double defenseMoveAngle = defense.defenseCalc(
      camera.ballAngle,
      homeGoalAngle,
      currentOffset,
      lineAngle,
      maxChordLength,
      crossLineState);

  Serial.println("Defense Move angle: " + String(defenseMoveAngle));

  if (defenseMoveAngle < 0)
  {
    movement->stop();
    return;
  }

  double desiredPerpendicularHeading = 0.0;
  bool desiredHeadingInBadZone = false;
  const double badZoneHeadingLimit = 53.0;
  // Serial.println("Desired Perpendicular Heading: " + String(desiredPerpendicularHeading));
  if (lineAngle != -5)
  {
    Serial.println("HELLOOooOoOooo");
    double relNormalA = Trig::wrapAngle(lineAngle);
    double relNormalB = Trig::wrapAngle(lineAngle + 180.0);
    double fieldNormalA = compassSensor.robotRelativeToField(relNormalA);
    double fieldNormalB = compassSensor.robotRelativeToField(relNormalB);
    bool normalAInBadZone = fabs(fieldNormalA) > badZoneHeadingLimit;
    bool normalBInBadZone = fabs(fieldNormalB) > badZoneHeadingLimit;

    if (normalAInBadZone != normalBInBadZone)
    {
      desiredPerpendicularHeading = normalAInBadZone ? fieldNormalB : fieldNormalA;
    }
    else
    {
      double chosenRelativeNormal = (fabs(relNormalA) <= fabs(relNormalB)) ? relNormalA : relNormalB;
      desiredPerpendicularHeading = compassSensor.robotRelativeToField(chosenRelativeNormal);
    }

    desiredHeadingInBadZone = fabs(desiredPerpendicularHeading) > badZoneHeadingLimit;
    Serial.println("Field Relative Desired Heading: " + String(desiredPerpendicularHeading));
  }

  if (desiredHeadingInBadZone) {
    Serial.println("YOU ARE APPROACHING A BAD ZONE");
    if ((desiredPerpendicularHeading >= badZoneHeadingLimit && abs(defenseMoveAngle - 90) <  30) || 
    (desiredPerpendicularHeading <= badZoneHeadingLimit && abs(defenseMoveAngle - 270) <  30)
    ) {
      movement->stop();
      return;
    }
  }

  Serial.println("Moving to Defense Position");
  Serial.println("Defense Move Angle: " + String(defenseMoveAngle));
  Serial.println("Desired Perpendicular Heading: " + String(desiredPerpendicularHeading));
  Serial.println("Desired Heading: " + String(desiredPerpendicularHeading));



  movement->movement(defenseMoveAngle, defenseSpeedFactor, desiredPerpendicularHeading, false);
}

void loop()
{
  if (modeControl == nullptr || movement == nullptr || offenseStateMachine == nullptr)
  {
    return;
  }

  modeControl->readCommands();

  // if (kRobotMode == RobotMode::Offense)
  // {
  //   offenseStateMachine->run(kConfiguredOffenseState);
  // } else {
  //   runDefense();
  // }

  runDefense();

  sendHeadingTelemetryToPi();
  linePCBComm.setRobotHeadingDegrees(compassSensor.currentOffset());
  linePCBComm.update();
  double lcdLineAngle = linePCBComm.getLineAngle();
  double lcdAvoidanceAngle = linePCBComm.getAvoidanceAngle();
  if (kRobotMode == RobotMode::Offense)
  {
    lcdLineAngle = offenseStateMachine->lineAngle();
    lcdAvoidanceAngle = offenseStateMachine->avoidanceAngle();
  }
  modeControl->sendTelemetry(lcdLineAngle, lcdAvoidanceAngle);
  // delay(1000);
}
