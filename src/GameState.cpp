#include <GameState.h>

#include <Cam.h>
#include <CompassSensor.h>
#include <LinePCBComm.h>
#include <ModeControl.h>
#include <Movement.h>
#include <RobotConfig.h>
#include <trig.h>

GameState::GameState(Cam& camera,
                     LinePCBComm& linePCBComm,
                     CompassSensor& compassSensor,
                     ModeControl& modeControl,
                     Movement& movement)
  : _camera(camera),
    _linePCBComm(linePCBComm),
    _compassSensor(compassSensor),
    _modeControl(modeControl),
    _movement(movement)
{
}

void GameState::update()
{
  // Camera must be read first: CamCalc() also pushes the latest LIDAR pose into
  // Movement::currentPose, which we snapshot below.
  _camera.CamCalc();

  ballAngle = _camera.ballAngle;
  ballDistance = _camera.ballDist;
  ballDerivative = _camera.derivative;
  ballSampleTime = _camera.sampleTime;
  predictedBallAngle = _camera.predictedBallAngle;

  lineAngle = _linePCBComm.getLineAngle();
  avoidanceAngle = _linePCBComm.getAvoidanceAngle();
  chordLength = _linePCBComm.getChordLength();
  crossLine = _linePCBComm.getCrossLine();

  heading = _compassSensor.currentOffset();
  pose = _movement.currentPose;
  pose.heading = heading;                 // ensure getAngle() uses the fresh heading
  _movement.currentPose.heading = heading; // keep Movement's copy consistent
  hasPose = (pose.x != -5 && pose.y != -5);

  goalIsBlue = _modeControl.state.goalIsBlue;
  cameraAttackGoal = goalIsBlue ? _camera.blueGoal : _camera.yellowGoal;
  cameraHomeGoal   = goalIsBlue ? _camera.yellowGoal : _camera.blueGoal;
}

double GameState::poseGoalAngle(double goalCenterY) const
{
  if (!hasPose)
  {
    return -5;
  }
  Point target;
  target.x = 0.0;
  target.y = goalCenterY;
  return Trig::normalize180(Trig::getAngle(pose, target));
}

double GameState::attackGoalAngle() const
{
  if (cameraAttackGoal != -5)
  {
    return cameraAttackGoal;
  }
  return poseGoalAngle(kAttackGoalCenterY);
}

bool GameState::hasAttackGoal() const
{
  return attackGoalAngle() != -5;
}

double GameState::homeGoalAngle() const
{
  if (cameraHomeGoal != -5)
  {
    return cameraHomeGoal;
  }
  return poseGoalAngle(kHomeGoalCenterY);
}

bool GameState::hasHomeGoal() const
{
  return homeGoalAngle() != -5;
}

double GameState::defenseBallAngle() const
{
  return _camera.selectedDefenseBallAngle();
}
