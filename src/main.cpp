#include <Arduino.h>
#include <math.h>

#include <Cam.h>
#include <Callibration.h>
#include <CompassSensor.h>
#include <GameState.h>
#include <GoalieCurveBoundary.h>
#include <LinePCBComm.h>
#include <ModeControl.h>
#include <Movement.h>
#include <OffenseStateMachine.h>
#include <RobotConfig.h>
#include <VirtualBoundary.h>
#include <orbit.h>
#include <Defense.h>
#include <DefenseStateMachine.h>
#include <trig.h>

// Configure the active offense mode here while the automatic transitions are
// still being developed.
constexpr OffenseState kConfiguredOffenseState = OffenseState::Orbit;

CompassSensor compassSensor;
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
DefenseStateMachine* defenseStateMachine = nullptr;
GameState* gameState = nullptr;
VirtualBoundaryBounds virtualBoundaryBounds = {
  virtualBoundaryMinX,
  virtualBoundaryMaxX,
  virtualBoundaryMinY,
  virtualBoundaryMaxY
};
VirtualBoundary virtualBoundary(virtualBoundaryBounds);
GoalieCurveBoundaryConfig goalieCurveBoundaryConfig = {
  goalieCurveMinOffsetMm,
  goalieCurveMaxOffsetMm,
  goalieCurveRampDistanceMm,
  goalieCurveHardMinOffsetMm,
  goalieCurveHardMaxOffsetMm
};
GoalieCurveBoundary goalieCurveBoundary(goalieCurveBoundaryConfig);
unsigned long lastPiHeadingTelemetryMs = 0;

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
  Serial3.print("T,heading=");
  Serial3.print(compassSensor.currentOffset());
  Serial3.println();
}

void setup()
{
  Serial.begin(115200);
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
  pinMode(27, INPUT);

  initializeDriveMotors();
  movement = new Movement(*FL, *FR, *BL, *BR, *dribbler, compassSensor);
  camera.setMovement(movement);
  modeControl = new ModeControl(Serial8, linePCBComm, compassSensor, *movement);
  camera.setModeControl(modeControl);
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

  gameState = new GameState(camera, linePCBComm, compassSensor, *modeControl, *movement);

  defenseStateMachine = new DefenseStateMachine(
    defense,
    *movement,
    *modeControl,
    compassSensor,
    goalieCurveBoundary,
    virtualBoundary);
}

void loop()
{
  if (modeControl == nullptr || movement == nullptr || offenseStateMachine == nullptr ||
      defenseStateMachine == nullptr || gameState == nullptr)
  {
    return;
  }

  modeControl->readCommands();
  linePCBComm.update();

  if (modeControl->state.lineCalibrationActive)
  {
    movement->stop();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
  }
  else
  {
    // Single per-loop snapshot of world state shared by both state machines.
    gameState->update();
    movement->kickBackground();

    if (modeControl->isOffenseMode())
    {
      offenseStateMachine->run(*gameState, kConfiguredOffenseState);
    }
    else
    {
      defenseStateMachine->run(*gameState);
    }
  }

  sendHeadingTelemetryToPi();
  double lcdLineAngle = linePCBComm.getLineAngle();
  double lcdAvoidanceAngle = linePCBComm.getAvoidanceAngle();

  modeControl->sendTelemetry(lcdLineAngle, lcdAvoidanceAngle,
                             movement->currentPose.x, movement->currentPose.y);
  if (robotDebugNoMoveMode)
  {
    delay(robotDebugLoopDelayMs);

    for (int i = 0; i < 5; i++) {
      Serial.println();
    }
  }
}

