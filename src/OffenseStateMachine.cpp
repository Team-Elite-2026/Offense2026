#include <Arduino.h>
#include <math.h>

#include <OffenseStateMachine.h>
#include <RobotConfig.h>

OffenseStateMachine::OffenseStateMachine(
  CompassSensor& compassSensor,
  Calibration& calibration,
  LinePCBComm& linePCBComm,
  Cam& camera,
  Orbit& orbit,
  Movement& movement,
  ModeControl& modeControl)
  : _compassSensor(compassSensor),
    _calibration(calibration),
    _linePCBComm(linePCBComm),
    _camera(camera),
    _orbit(orbit),
    _movement(movement),
    _modeControl(modeControl)
{
}

void OffenseStateMachine::run(OffenseState configuredState, const Point& spinShotTargetPose)
{
  if (_modeControl.state.lineCalibrationActive)
  {
    _movement.stop();
    _calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    return;
  }

  updateVisionAndLineState();
  _movement.kickBackground();
  printDebugState();

  if (!_modeControl.isStartEnabled())
  {
    _movement.stop();
    return;
  }

  // if (_lineAngle != -5)
  // {
  //   runLineAvoidance();
  //   return;
  // }

  switch (configuredState)
  {
    case OffenseState::SpinShot:
      runSpinShotState(spinShotTargetPose);
      break;
    case OffenseState::Orbit:
    default:
      runOrbitState();
      break;
  }
}

void OffenseStateMachine::updateVisionAndLineState()
{
  _linePCBComm.setRobotHeadingDegrees(_compassSensor.currentOffset());
  _linePCBComm.update();
  _camera.CamCalc();

  _lineAngle = _linePCBComm.getLineAngle();
  _goalAngle = _modeControl.state.goalIsBlue ? _camera.blueGoal : _camera.yellowGoal;
  _aimingGoal = _goalAngle != -5;
  if (!_aimingGoal)
  {
    _goalAngle = 0;
  }

  _orbitAngle = _orbit.CalculateRobotAngle(
    _camera.ballAngle,
    _camera.ballDist,
    _camera.derivative,
    _camera.sampleTime);
}

void OffenseStateMachine::runLineAvoidance()
{
  _avoidanceAngle = _linePCBComm.getAvoidanceAngle();
  Serial.println("Avoidance angle: " + String(_avoidanceAngle));
  _movement.movement(_avoidanceAngle, lineAvoidanceSpeed, 0, false);
}

void OffenseStateMachine::runOrbitState()
{
  if (_modeControl.doWeHaveBall())
  {
    if (fabs(_goalAngle) < 5)
    {
      _movement.kick();
    }
    return;
  }

  if (_camera.ballAngle != -5)
  {
    _movement.movement(_orbitAngle, offenseSpeedFactor, _goalAngle, _aimingGoal);
    return;
  }

  _movement.stop();
}

void OffenseStateMachine::runSpinShotState(const Point& spinShotTargetPose)
{
  _movement.PlanToPose(spinShotTargetPose);
}

void OffenseStateMachine::printDebugState() const
{
  Serial.println("Line Angle: " + String(_lineAngle));
  Serial.println("Robot Angle: " + String(_orbitAngle));
  Serial.println("Ball Angle: " + String(_camera.ballAngle));
  Serial.println("Goal Angle: " + String(_goalAngle));
  Serial.println("Ball dist:" + String(_camera.ballDist));
  Serial.println("Robot Heading: " + String(_compassSensor.currentOffset()));
}
