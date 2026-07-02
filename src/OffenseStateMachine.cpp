#include <Arduino.h>
#include <math.h>

#include <OffenseStateMachine.h>
#include <RobotConfig.h>
#include <trig.h>

namespace {
// Maps distance-to-target (cm) to an approach speed factor. Negative distance
// (target unknown) falls back to full offense speed.
double orbitApproachSpeed(double angle)
{
  // // make this a sin function
  // if (distToTarget < 0.0)
  // {
  //   return offenseSpeedFactor;
  // }
  // double t = distToTarget / kOrbitDecelRangeCm;
  // if (t > 1.0) t = 1.0;
  // if (t < 0.0) t = 0.0;
  // return kOrbitCaptureSpeed + (offenseSpeedFactor - kOrbitCaptureSpeed) * t;

  return kMinOrbitCaptureSpeed + (offenseSpeedFactor - kMinOrbitCaptureSpeed) * fabs(Trig::Cos(Trig::normalize180(angle)));
}

double pwmToFactor(double pwm)
{
  return pwm / kPwmMax;
}

double blendOrbitWithLineAvoidance(double orbitAngle,
                                   double avoidanceAngle,
                                   double chordLengthNormalized)
{
  double chord = (chordLengthNormalized < 0.0)
      ? kOrbitLineDefaultChordLength
      : Trig::clamp(chordLengthNormalized, 0.0, 1.0);

  double orbitX = Trig::Sin(orbitAngle);
  double orbitY = Trig::Cos(orbitAngle);
  double avoidX = Trig::Sin(avoidanceAngle);
  double avoidY = Trig::Cos(avoidanceAngle);

  double outwardProjection = (orbitX * avoidX) + (orbitY * avoidY);
  double tangentX = orbitX - (outwardProjection * avoidX);
  double tangentY = orbitY - (outwardProjection * avoidY);
  double outwardGain =
      fmax(outwardProjection, 0.0) +
      kOrbitLineMinOutwardGain +
      (kOrbitLineChordOutwardGain * chord);

  double blendedX = tangentX + (outwardGain * avoidX);
  double blendedY = tangentY + (outwardGain * avoidY);
  if ((blendedX * blendedX + blendedY * blendedY) < kOrbitLineMinVectorMagnitudeSq)
  {
    return Trig::normalize360(avoidanceAngle);
  }

  return Trig::angleFromVector(blendedX, blendedY);
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

  if (_lineAngle != -5)
  {
    runLineAvoidance();
    return;
  }

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
  _camera.CamCalc();

  _movement.currentPose.heading = _compassSensor.currentOffset();
  _lineAngle = _linePCBComm.getLineAngle();
  _goalAngle = _modeControl.state.goalIsBlue ? _camera.blueGoal : _camera.yellowGoal;
  _aimingGoal = _goalAngle != -5;
  if (!_aimingGoal)
  {
    _goalAngle = Trig::getAngle(
        _movement.currentPose,
        {kFallbackGoalX, kFallbackGoalY, kFallbackGoalHeading});
    _aimingGoal = true;
  }

  _orbitAngle = _orbit.CalculateRobotAngle(
    _camera.ballAngle,
    _camera.ballDist,
    _camera.derivative,
    _camera.sampleTime,
    _goalAngle,
    _aimingGoal);
}

void OffenseStateMachine::runLineAvoidance()
{
  _avoidanceAngle = _linePCBComm.getAvoidanceAngle();
  if (_avoidanceAngle < 0.0)
  {
    _movement.stop();
    return;
  }

  double movementAngle = _avoidanceAngle;
  if (_camera.ballAngle != -5 && _orbitAngle >= 0.0)
  {
    movementAngle = blendOrbitWithLineAvoidance(
        _orbitAngle,
        _avoidanceAngle,
        _linePCBComm.getChordLength());
  }

  // Serial.println("Avoidance angle: " + String(_avoidanceAngle));
  // Serial.println("Blended orbit/line angle: " + String(movementAngle));
  _movement.movement(movementAngle, kOrbitLineAvoidanceSpeed, _goalAngle, true);
}

void OffenseStateMachine::runOrbitState()
{
  // ballDist is in cm; -5 means the ball is not currently seen.
  bool ballClose = (_camera.ballDist != -5) && (_camera.ballDist < kBallCloseCm);

  // Run the dribbler to draw the ball in once we are close.
  _movement.setDribbler(ballClose ? pwmToFactor(kDribblerApproachPwm) : 0.0);

  if (_modeControl.doWeHaveBall())
  {
    _movement.kick();
  }

  if (_camera.ballAngle != -5)
  {
    _orbitLostBallTimer = 0;
    _orbitLostBallTimerActive = false;
    double approachSpeed = offenseSpeedFactor;
    if (Trig::angularDistance(_camera.ballAngle, 0.0) > kOrbitForwardDeadbandDeg)
    {
      approachSpeed = orbitApproachSpeed(_orbitAngle);
    }
    _movement.movement(_orbitAngle, approachSpeed, _goalAngle, _aimingGoal);
    return;
  }

  if (!_orbitLostBallTimerActive)
  {
    _orbitLostBallTimer = 0;
    _orbitLostBallTimerActive = true;
  }

  if (_orbitLostBallTimer >= kOrbitLostBallCenterDelayMs)
  {
    _movement.PlanToPose({
        kOrbitLostBallRecoveryX,
        kOrbitLostBallRecoveryY,
        kOrbitLostBallRecoveryHeading});
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

  double spinFactor;
  if (_aimingGoal)
  {
    // Goal is in view: drive the spin with the same PID the heading correction
    // uses, so the spin speed scales with the heading error to the goal and the
    // sign automatically turns us toward it.  _goalAngle is already robot/camera
    // relative (0 = goal dead ahead), which is exactly what findCorrectionRelOffset
    // expects.  Cap the magnitude so a large error cannot spin us violently.
    spinFactor = _movement.findCorrectionRelOffset(_goalAngle);
    spinFactor = fmax(-kSpinShotMaxSpeed, fmin(kSpinShotMaxSpeed, spinFactor));
  }
  else
  {
    // Goal not yet in view: sweep at a fixed speed in the pose's search direction
    // until the camera picks the goal up.
    spinFactor = _targetShotPose->spinSign * kSpinSearchSpeed;
  }
  _movement.spin(spinFactor);

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
  // The pose frame is attack-relative (+y is always the attacked goal), so the
  // same left/right pair works no matter which physical goal we attack -- no
  // goalIsBlue branch needed.  Pick whichever side we are closer to.
  double distLeft  = Trig::getDist(_movement.currentPose, kShotPoseLeft.pose);
  double distRight = Trig::getDist(_movement.currentPose, kShotPoseRight.pose);

  return (distRight <= distLeft) ? kShotPoseRight : kShotPoseLeft;
}

void OffenseStateMachine::resetSequence()
{
  _activeState = OffenseState::Orbit;
  _targetShotPose = nullptr;
  _orbitLostBallTimer = 0;
  _orbitLostBallTimerActive = false;
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
