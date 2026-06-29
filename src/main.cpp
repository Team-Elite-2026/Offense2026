#include <Arduino.h>
#include <math.h>

#include <Cam.h>
#include <Callibration.h>
#include <CompassSensor.h>
#include <GoalieCurveBoundary.h>
#include <LinePCBComm.h>
#include <ModeControl.h>
#include <Movement.h>
#include <OffenseStateMachine.h>
#include <RobotConfig.h>
#include <VirtualBoundary.h>
#include <orbit.h>
#include <Defense.h>
#include <trig.h>

// Configure the active offense mode here while the automatic transitions are
// still being developed.
constexpr OffenseState kConfiguredOffenseState = OffenseState::Orbit;

constexpr unsigned long kPiHeadingTelemetryIntervalMs = 100;

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
}

double getHomeGoalAngle() {
  if (modeControl->isGoalBlueSelected()) {
    return camera.yellowGoal;
  } 
  return camera.blueGoal;
}

void runDefense()
{

     if (modeControl->state.lineCalibrationActive)
  {
    movement->stop();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    return;
  }
  
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

  // Serial.println("Line Angle: " + String(lineAngle));
  // Serial.println("Ball Angle: " + String(camera.ballAngle));
  // Serial.println("Home Goal Angle: " + String(homeGoalAngle));
  // Serial.println("Max Normalized Activated Sensor Distance: " + String(maxChordLength));
  // Serial.println("Cross Line: " + String(crossLineState ? "true" : "false"));
  // Serial.println("Current offset: " + String(currentOffset));
  // Serial.println("Ball Angle: " + String(camera.ballAngle));

  if (!modeControl->isStartEnabled())
  {
    movement->stop();
    return;
  }

  bool hasPose = movement->currentPose.x != -5 && movement->currentPose.y != -5;
  if (hasPose)
  {
    double virtualBoundaryAngle = -1.0;
    bool outsideVirtualBoundary =
        virtualBoundary.getAvoidanceAngle(movement->currentPose, currentOffset, virtualBoundaryAngle);

    if (outsideVirtualBoundary)
    {
      if (virtualBoundaryDebugEnabled)
      {
        Serial.println("Virtual Boundary Active");
        Serial.println("Virtual Boundary Pose X: " + String(movement->currentPose.x));
        Serial.println("Virtual Boundary Pose Y: " + String(movement->currentPose.y));
        Serial.println("Virtual Boundary Move Angle: " + String(virtualBoundaryAngle));
      }

      if (virtualBoundaryDriveEnabled)
      {
        movement->movement(virtualBoundaryAngle, virtualBoundaryAvoidanceSpeed, currentOffset, false);
        return;
      }
    }
  }

  GoalieCurveBoundaryResult goalieCurveResult;
  bool hasGoalieCurveResult = false;
  if (hasPose)
  {
    goalieCurveResult = goalieCurveBoundary.evaluate(movement->currentPose, currentOffset);
    hasGoalieCurveResult = true;
  }

  bool goalieCurveCanDrive = hasGoalieCurveResult &&
                             goalieCurveDriveEnabled &&
                             goalieCurveResult.hasCorrection();
  double defenseBallAngle = camera.selectedDefenseBallAngle();

  if (defenseBallAngle == -5)
  {
    if (goalieCurveCanDrive && goalieCurveResult.hardRecovery)
    {
      if (goalieCurveDebugEnabled)
      {
        goalieCurveBoundary.printDebug(goalieCurveResult, goalieCurveResult.correctionRobotAngle);
      }
      movement->movement(goalieCurveResult.correctionRobotAngle,
                         defenseSpeedFactor,
                         currentOffset,
                         false);
      return;
    }

    movement->stop();
    return;
  }

  bool ballInDeadband = Trig::angularDistance(defenseBallAngle, 0.0) <= defenseBallDeadbandDegrees;
  bool defenseMovementActive = !ballInDeadband;
  double defenseMoveAngle = -1.0;

  if (defenseMovementActive)
  {
    if (homeGoalAngle == -5)
    {
      defenseMoveAngle = Trig::normalize360(defenseBallAngle);
    }
    else
    {
      defenseMoveAngle = defense.defenseCalc(
          defenseBallAngle,
          homeGoalAngle,
          currentOffset,
          lineAngle,
          maxChordLength,
          crossLineState);
    }
  }

  // Serial.println("Raw Ball Angle: " + String(camera.ballAngle));
  // Serial.println("Predicted Ball Angle: " + String(camera.predictedBallAngle));
  // Serial.println("Defense Ball Angle: " + String(defenseBallAngle));
  // Serial.println("Defense Move angle: " + String(defenseMoveAngle));

  if (defenseMovementActive && defenseMoveAngle < 0)
  {
    if (goalieCurveCanDrive && goalieCurveResult.hardRecovery)
    {
      if (goalieCurveDebugEnabled)
      {
        goalieCurveBoundary.printDebug(goalieCurveResult, goalieCurveResult.correctionRobotAngle);
      }
      movement->movement(goalieCurveResult.correctionRobotAngle,
                         defenseSpeedFactor,
                         currentOffset,
                         false);
      return;
    }

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

  bool hasMoveCommand = defenseMovementActive;
  double finalMoveAngle = defenseMoveAngle;

  if (hasGoalieCurveResult && goalieCurveResult.hasCorrection())
  {
    double blendedRobotAngle = hasMoveCommand
        ? goalieCurveBoundary.blendWithDefenseAngle(
            defenseMoveAngle,
            currentOffset,
            goalieCurveBoundaryWeight,
            goalieCurveResult)
        : goalieCurveResult.correctionRobotAngle;

    if (goalieCurveDebugEnabled)
    {
      goalieCurveBoundary.printDebug(goalieCurveResult, blendedRobotAngle);
    }

    if (goalieCurveDriveEnabled)
    {
      finalMoveAngle = goalieCurveResult.hardRecovery
          ? goalieCurveResult.correctionRobotAngle
          : blendedRobotAngle;
      hasMoveCommand = true;
    }
  }

  if (!hasMoveCommand)
  {
    movement->stop();
    return;
  }

  if (desiredHeadingInBadZone)
  {
    Serial.println("YOU ARE APPROACHING A BAD ZONE");
    if ((desiredPerpendicularHeading >= badZoneHeadingLimit && Trig::angularDistance(finalMoveAngle, 90.0) < 30.0) ||
        (desiredPerpendicularHeading <= -badZoneHeadingLimit && Trig::angularDistance(finalMoveAngle, 270.0) < 30.0))
    {
      movement->stop();
      return;
    }
  }

  movement->movement(finalMoveAngle, defenseSpeedFactor, desiredPerpendicularHeading, false);
}

void loop()
{
  if (modeControl == nullptr || movement == nullptr || offenseStateMachine == nullptr)
  {
    return;
  }

  modeControl->readCommands();

  if (modeControl->isOffenseMode())
  {
    offenseStateMachine->run(kConfiguredOffenseState);
  } else {
    runDefense();
  }

  sendHeadingTelemetryToPi();
  linePCBComm.update();
  double lcdLineAngle = linePCBComm.getLineAngle();
  double lcdAvoidanceAngle = linePCBComm.getAvoidanceAngle();

  modeControl->sendTelemetry(lcdLineAngle, lcdAvoidanceAngle,
                             movement->currentPose.x, movement->currentPose.y);
  // delay(1000);
}
