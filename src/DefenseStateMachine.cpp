#include <Arduino.h>
#include <math.h>

#include <CompassSensor.h>
#include <Defense.h>
#include <DefenseStateMachine.h>
#include <GameState.h>
#include <GoalieCurveBoundary.h>
#include <ModeControl.h>
#include <Movement.h>
#include <RobotConfig.h>
#include <VirtualBoundary.h>
#include <trig.h>

DefenseStateMachine::DefenseStateMachine(Defense& defense,
                                         Movement& movement,
                                         ModeControl& modeControl,
                                         CompassSensor& compassSensor,
                                         GoalieCurveBoundary& goalieCurveBoundary,
                                         VirtualBoundary& virtualBoundary)
  : _defense(defense),
    _movement(movement),
    _modeControl(modeControl),
    _compassSensor(compassSensor),
    _goalieCurveBoundary(goalieCurveBoundary),
    _virtualBoundary(virtualBoundary)
{
}

bool DefenseStateMachine::shouldUseGoalieCurveBoundary(double lineAngle) const
{
  if (lineAngle == -5)
  {
    return true;
  }

  return fabs(Trig::wrapAngle(lineAngle)) <= goalieCurveLineAngleWindowDegrees;
}

void DefenseStateMachine::run(GameState& gameState)
{
  // Line calibration, kickBackground(), and gameState.update() are handled by the
  // main loop before dispatching here, so they are not repeated.
  const double lineAngle      = gameState.lineAngle;
  const double maxChordLength = gameState.chordLength;
  const bool   crossLineState = gameState.crossLine;
  const double currentOffset  = gameState.heading;
  const bool   hasPose        = gameState.hasPose;
  double homeGoalAngle        = gameState.homeGoalAngle();

  if (lineAngle != -5)
  {
    Serial.println("Avoidance Angle: " + String(gameState.avoidanceAngle));
  }

  Serial.println("Line Angle: " + String(lineAngle));
  Serial.println("Ball Angle: " + String(gameState.ballAngle));
  // Serial.println("Home Goal Angle: " + String(homeGoalAngle));
  // Serial.println("Max Normalized Activated Sensor Distance: " + String(maxChordLength));
  // Serial.println("Cross Line: " + String(crossLineState ? "true" : "false"));
  // Serial.println("Current offset: " + String(currentOffset));

  if (!_modeControl.isStartEnabled())
  {
    _movement.stop();
    return;
  }

  if (hasPose)
  {
    double virtualBoundaryAngle = -1.0;
    bool outsideVirtualBoundary =
        _virtualBoundary.getAvoidanceAngle(_movement.currentPose, currentOffset, virtualBoundaryAngle);

    if (outsideVirtualBoundary)
    {
      if (virtualBoundaryDebugEnabled)
      {
        Serial.println("Virtual Boundary Active");
        Serial.println("Virtual Boundary Pose X: " + String(_movement.currentPose.x));
        Serial.println("Virtual Boundary Pose Y: " + String(_movement.currentPose.y));
        Serial.println("Virtual Boundary Move Angle: " + String(virtualBoundaryAngle));
      }

      if (virtualBoundaryDriveEnabled)
      {
        _movement.movement(virtualBoundaryAngle, virtualBoundaryAvoidanceSpeed, currentOffset, false);
        return;
      }
    }
  }

  GoalieCurveBoundaryResult goalieCurveResult;
  bool hasGoalieCurveResult = false;
  bool useGoalieCurveBoundary = shouldUseGoalieCurveBoundary(lineAngle);
  if (hasPose && useGoalieCurveBoundary)
  {
    goalieCurveResult = _goalieCurveBoundary.evaluate(_movement.currentPose, currentOffset);
    hasGoalieCurveResult = true;

    if (goalieCurveDebugEnabled)
    {
      _goalieCurveBoundary.printDebug(goalieCurveResult, goalieCurveResult.correctionRobotAngle);
    }
  }

  double defenseBallAngle = gameState.defenseBallAngle();

  if (defenseBallAngle == -5)
  {
    _movement.stop();
    return;
  }

  bool ballInDeadband = Trig::angularDistance(defenseBallAngle, 0.0) <= defenseBallDeadbandDegrees;
  bool defenseMovementActive = !ballInDeadband;
  double defenseMoveAngle = -1.0;

  // homeGoalAngle is camera-derived when the goal is seen, else pose-derived
  // (gameState.homeGoalAngle()), and only -5 when neither is available.
  if (defenseMovementActive && homeGoalAngle != -5)
  {
    defenseMoveAngle = _defense.defenseCalc(
        defenseBallAngle,
        homeGoalAngle,
        currentOffset,
        lineAngle,
        maxChordLength,
        crossLineState);
  }

  Serial.println("Home Goal Angle: " + String(homeGoalAngle));
  // Serial.println("Raw Ball Angle: " + String(gameState.ballAngle));
  // Serial.println("Predicted Ball Angle: " + String(gameState.predictedBallAngle));
  // Serial.println("Defense Ball Angle: " + String(defenseBallAngle));
  // Serial.println("Defense Move angle: " + String(defenseMoveAngle));

  if (defenseMovementActive && defenseMoveAngle < 0)
  {
    _movement.stop();
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
    double fieldNormalA = _compassSensor.robotRelativeToField(relNormalA);
    double fieldNormalB = _compassSensor.robotRelativeToField(relNormalB);
    bool normalAInBadZone = fabs(fieldNormalA) > badZoneHeadingLimit;
    bool normalBInBadZone = fabs(fieldNormalB) > badZoneHeadingLimit;

    if (normalAInBadZone != normalBInBadZone)
    {
      desiredPerpendicularHeading = normalAInBadZone ? fieldNormalB : fieldNormalA;
    }
    else
    {
      double chosenRelativeNormal = (fabs(relNormalA) <= fabs(relNormalB)) ? relNormalA : relNormalB;
      desiredPerpendicularHeading = _compassSensor.robotRelativeToField(chosenRelativeNormal);
    }

    desiredHeadingInBadZone = fabs(desiredPerpendicularHeading) > badZoneHeadingLimit;
    Serial.println("Field Relative Desired Heading: " + String(desiredPerpendicularHeading));
  }

  bool hasMoveCommand = defenseMovementActive;
  double finalMoveAngle = defenseMoveAngle;

  if (hasGoalieCurveResult && goalieCurveResult.hasCorrection())
  {
    double blendedRobotAngle = hasMoveCommand
        ? _goalieCurveBoundary.blendWithDefenseAngle(
            defenseMoveAngle,
            currentOffset,
            goalieCurveBoundaryWeight,
            goalieCurveResult)
        : goalieCurveResult.correctionRobotAngle;

    if (goalieCurveDebugEnabled)
    {
      _goalieCurveBoundary.printDebug(goalieCurveResult, blendedRobotAngle);
    }

    if (goalieCurveDriveEnabled)
    {
      finalMoveAngle = blendedRobotAngle;
      hasMoveCommand = true;
    }
  }

  if (!hasMoveCommand)
  {
    _movement.stop();
    return;
  }

  if (desiredHeadingInBadZone)
  {
    Serial.println("YOU ARE APPROACHING A BAD ZONE");
    if ((desiredPerpendicularHeading >= badZoneHeadingLimit && Trig::angularDistance(finalMoveAngle, 90.0) < 30.0) ||
        (desiredPerpendicularHeading <= -badZoneHeadingLimit && Trig::angularDistance(finalMoveAngle, 270.0) < 30.0))
    {
      _movement.stop();
      return;
    }
  }

  Serial.println("Final Move Angle: " + String(finalMoveAngle));
  _movement.movement(finalMoveAngle, defenseSpeedFactor, desiredPerpendicularHeading, false);
}
