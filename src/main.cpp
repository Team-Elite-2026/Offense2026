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
Orbit orbit(1);
Cam camera;
LinePCBComm linePCBComm(Serial2);
ModeControl* modeControl = nullptr;
OffenseStateMachine* offenseStateMachine = nullptr;
unsigned long lastPiHeadingTelemetryMs = 0;

static void initializeDriveMotors()
{
  pinMode(selectionPin, INPUT);
  const bool useDefaultMotorLayout = digitalRead(selectionPin) == LOW;

  if (useDefaultMotorLayout)
  {
    FL = new Motor(pincontrolFLA, pincontrolFLB, pinspeedFL);
    FR = new Motor(pincontrolFRA, pincontrolFRB, pinspeedFR);
    BL = new Motor(pincontrolRLA, pincontrolRLB, pinspeedRL);
    BR = new Motor(pincontrolRRA, pincontrolRRB, pinspeedRR);
  }
  else
  {
    BR = new Motor(pincontrolFLA, pincontrolFLB, pinspeedFL);
    FR = new Motor(pincontrolFRA, pincontrolFRB, pinspeedFR);
    FL = new Motor(pincontrolRLB, pincontrolRLA, pinspeedRL);
    BL = new Motor(pincontrolRRB, pincontrolRRA, pinspeedRR);
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

  // compassSensor.callibrate();
  compassSensor.begin();
  calibration.calibrateCompassSensor();
  linePCBComm.begin(1000000);

  initializeDriveMotors();
  movement = new Movement(*FL, *FR, *BL, *BR, *dribbler, compassSensor);
  camera.setMovement(movement);
  modeControl = new ModeControl(Serial8, linePCBComm, compassSensor, *movement, kRobotMode);
  modeControl->begin(115200);
  offenseStateMachine = new OffenseStateMachine(
    compassSensor,
    calibration,
    linePCBComm,
    camera,
    orbit,
    *movement,
    *modeControl);
}

void runDefense()
{
  if (switches.calibration())
  {
    movement->stop();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    return;
  }

  camera.CamCalc();
  movement->kickBackground();

  if (!switches.start())
  {
    movement->stop();
    return;
  }

  if (camera.ballAngle == -5)
  {
    movement->stop();
    return;
  }

  movement->movement(camera.ballAngle, defenseSpeedFactor, 0, false);
}

void loop()
{
  if (modeControl == nullptr || movement == nullptr || offenseStateMachine == nullptr)
  {
    return;
  }

  modeControl->readCommands();

  if (kRobotMode == RobotMode::Offense)
  {
    offenseStateMachine->run(kConfiguredOffenseState);
  } else {
    runDefense();
  }

  sendHeadingTelemetryToPi();
  linePCBComm.update();
  // delay(1000);
}
