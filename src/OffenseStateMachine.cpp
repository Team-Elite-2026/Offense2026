#include <Arduino.h>
#include <math.h>

#include <OffenseStateMachine.h>
#include <RobotConfig.h>
#include <trig.h>

namespace {
// PathPlan: how close (mm) to the shot pose counts as "arrived".
constexpr double kArrivalMarginMm = 30.0;

// SpinShot: spin magnitude (sign comes from the chosen shot pose) and the
// |goalAngle| (degrees) under which we are aimed well enough to kick.
constexpr double kSpinSpeed          = 0.20;
constexpr double kGoalAlignedDegrees = 12.0;

// Orbit ball approach: when the ball is closer than this (cm) slow down and
// start the dribbler to draw the ball in.
constexpr double kBallCloseCm       = 15.0;
constexpr double kBallApproachSpeed = 0.10;

// Dribbler PWM setpoints (0..255), converted to motor speed factors.
constexpr double kDribblerApproachPwm = 96.0;   // closing on the ball in orbit
constexpr double kDribblerTravelPwm   = 145.0;  // carrying the ball in PathPlan
constexpr double kDribblerMaxPwm      = 255.0;  // spin-up, spin shot, and kick

// DribblerToKick: hold the dribbler at full speed this long before spinning.
constexpr unsigned long kDribblerSpinUpMs = 250;

double pwmToFactor(double pwm)
{
  return pwm / 255.0;
}

// Returns the name of the given offense state as a string for debugging.
const char* stateName(OffenseState state)
{
  switch (state)
  {
    case OffenseState::Orbit:          return "Orbit";
    case OffenseState::PathPlan:       return "PathPlan";
    case OffenseState::DribblerToKick: return "DribblerToKick";
    case OffenseState::SpinShot:       return "SpinShot";
    case OffenseState::Kick:           return "Kick";
  }
  return "Unknown";
}
}

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

void OffenseStateMachine::run(OffenseState configuredState)
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
    _movement.setDribbler(0);
    resetSequence();
    return;
  }

  // if (_lineAngle != -5)
  // {
  //   runLineAvoidance();
  //   return;
  // }

  switch (configuredState)
  {
    case OffenseState::Orbit:
      runOrbitState();
      break;
    default:
      runShootSequence();
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
  // ballDist is in cm; -5 means the ball is not currently seen.
  bool ballClose = (_camera.ballDist != -5) && (_camera.ballDist < kBallCloseCm);

  // Run the dribbler to draw the ball in once we are close.
  _movement.setDribbler(ballClose ? pwmToFactor(kDribblerApproachPwm) : 0.0);

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
    // Slow way down for the final approach so we can capture the ball.
    double speedFactor = ballClose ? kBallApproachSpeed : offenseSpeedFactor;
    _movement.movement(_orbitAngle, speedFactor, _goalAngle, _aimingGoal);
    return;
  }

  _movement.stop();
}

void OffenseStateMachine::runShootSequence()
{
  switch (_activeState)
  {
    case OffenseState::PathPlan:
      runPathPlanState();
      break;
    case OffenseState::DribblerToKick:
      runDribblerToKickState();
      break;
    case OffenseState::SpinShot:
      runSpinShotState();
      break;
    case OffenseState::Kick:
      runKickState();
      break;
    default:
      _activeState = OffenseState::PathPlan;
      runPathPlanState();
      break;
  }
}

void OffenseStateMachine::runPathPlanState()
{
  // Lock in the closest shot pose (for the goal we are attacking) the first
  // time we enter PathPlan so the target does not flip while we drive to it.
  if (_targetShotPose == nullptr)
  {
    _targetShotPose = &selectNearestShotPose();
  }

  // Carry the ball at travel speed while driving to the shot pose.
  _movement.setDribbler(pwmToFactor(kDribblerTravelPwm));
  _movement.PlanToPose(_targetShotPose->pose);

  // Arrived -> start the dribbler spin-up before the shot.
  double distToTarget = Trig::getDist(_movement.currentPose, _targetShotPose->pose);
  if (distToTarget <= kArrivalMarginMm)
  {
    _dribblerToKickTimer = 0;
    _activeState = OffenseState::DribblerToKick;
  }
}

void OffenseStateMachine::runDribblerToKickState()
{
  // Hold position and run the dribbler at full speed for a fixed time.
  _movement.stop();
  _movement.setDribbler(pwmToFactor(kDribblerMaxPwm));

  if (_dribblerToKickTimer >= kDribblerSpinUpMs)
  {
    _activeState = OffenseState::SpinShot;
  }
}

void OffenseStateMachine::runSpinShotState()
{
  // Keep the ball pinned at full dribble and spin toward the goal.
  _movement.setDribbler(pwmToFactor(kDribblerMaxPwm));
  _movement.spin(_targetShotPose->spinSign * kSpinSpeed);

  // Kick only once we actually see the goal and are aimed within tolerance.
  if (_aimingGoal && fabs(_goalAngle) < kGoalAlignedDegrees)
  {
    _activeState = OffenseState::Kick;
  }
}

void OffenseStateMachine::runKickState()
{
  // Stop spinning/translating, hold the ball, and fire the kicker.
  _movement.stop();
  _movement.setDribbler(pwmToFactor(kDribblerMaxPwm));
  _movement.kick();
}

const ShotPose& OffenseStateMachine::selectNearestShotPose() const
{
  // Mode (goalIsBlue) decides which pair of shot poses we choose between.
  const ShotPose& poseA = _modeControl.state.goalIsBlue ? kBlueShotPoseA : kYellowShotPoseA;
  const ShotPose& poseB = _modeControl.state.goalIsBlue ? kBlueShotPoseB : kYellowShotPoseB;

  double distA = Trig::getDist(_movement.currentPose, poseA.pose);
  double distB = Trig::getDist(_movement.currentPose, poseB.pose);

  return (distA <= distB) ? poseA : poseB;
}

void OffenseStateMachine::resetSequence()
{
  _activeState = OffenseState::PathPlan;
  _targetShotPose = nullptr;
}

void OffenseStateMachine::printDebugState() const
{
  Serial.println("State: " + String(stateName(_activeState)));
  Serial.println("Line Angle: " + String(_lineAngle));
  Serial.println("Robot Angle: " + String(_orbitAngle));
  Serial.println("Ball Angle: " + String(_camera.ballAngle));
  Serial.println("Goal Angle: " + String(_goalAngle));
  Serial.println("Ball dist:" + String(_camera.ballDist));
  Serial.println("Robot Heading: " + String(_compassSensor.currentOffset()));
}
