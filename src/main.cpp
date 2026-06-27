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

// LidarLocalizer returns field-corner-origin coordinates in millimeters:
// x = 0..1820 across field width, y = 0..2430 along field height.
constexpr double kFieldWidthMm = 1820.0;
constexpr double kFieldHeightMm = 2430.0;
const Point kSpinShotTargetPose = {kFieldWidthMm * 0.5, kFieldHeightMm * 0.5, 0.0};

RobotMode kRobotMode = defaultRobotMode;

CompassSensor compassSensor;
Switch switches;
Calibration calibration(compassSensor);
Motor* FL = nullptr;
Motor* FR = nullptr;
Motor* BL = nullptr;
Motor* BR = nullptr;
Movement* movement = nullptr;
Orbit orbit(1);
Cam camera;
LinePCBComm linePCBComm(Serial2);
ModeControl* modeControl = nullptr;
OffenseStateMachine* offenseStateMachine = nullptr;

static void initializeDriveMotors()
{
  pinMode(selectionPin, INPUT);
  const bool useDefaultMotorLayout = digitalRead(selectionPin) == HIGH;

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
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Testing Run");
  Serial3.begin(2000000);

  compassSensor.begin();
  compassSensor.callibrate();
  linePCBComm.begin(1000000);

  initializeDriveMotors();
  movement = new Movement(*FL, *FR, *BL, *BR, compassSensor);
  camera.setMovement(movement);
  modeControl = new ModeControl(Serial8, linePCBComm, compassSensor, *movement, kRobotMode);
  modeControl->begin(115200);
  modeControl->applyRobotModeSettings();
  offenseStateMachine = new OffenseStateMachine(
    compassSensor,
    calibration,
    linePCBComm,
    camera,
    orbit,
    *movement,
    *modeControl);
}

// void runDefense()
// {
//   if (switches.calibration())
//   {
//     movement->stop();
//     calibration.calibrateCompassSensor();
//     Serial.println("Calibrating");
//     return;
//   }
//
//   camera.CamCalc();
//   movement->kickBackground();
//
//   if (!switches.start())
//   {
//     movement->stop();
//     return;
//   }
//
//   if (camera.ballAngle == -5)
//   {
//     movement->stop();
//     return;
//   }
//
//   movement->movement(camera.ballAngle, defenseSpeedFactor, 0, false);
// }

void loop()
{
  if (modeControl == nullptr || movement == nullptr || offenseStateMachine == nullptr)
  {
    return;
  }

  modeControl->readCommands();

  if (kRobotMode == RobotMode::Offense)
  {
    offenseStateMachine->run(kConfiguredOffenseState, kSpinShotTargetPose);
  }

  modeControl->sendTelemetry();
}
