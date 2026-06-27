#ifndef OFFENSE_STATE_MACHINE_H
#define OFFENSE_STATE_MACHINE_H

#include <Arduino.h>

#include <Callibration.h>
#include <Cam.h>
#include <CompassSensor.h>
#include <LinePCBComm.h>
#include <ModeControl.h>
#include <Movement.h>
#include <orbit.h>

enum class OffenseState
{
  Orbit,
  PathPlan,
  DribblerToKick,
  SpinShot,
  Kick,
};

// Field origin is the top-left corner, +x to the right, +y downward, in mm.
constexpr double kFieldWidthMm  = 1820.0;
constexpr double kFieldHeightMm = 2430.0;

// A shooting pose plus the spin direction to use once the robot arrives.
// spinSign is -1 (spin left) or +1 (spin right); used by SpinShot.
struct ShotPose
{
  Point pose;
  double spinSign;
};

// The four shooting poses sit 450mm in x and 670mm in y inward from each field
// corner. The two poses at y = 670 (nearer the top) are used when attacking the
// yellow goal; the two at y = kFieldHeightMm - 670 are used for the blue goal.
// All four face a heading of 180 degrees.
constexpr double kShotPoseXOffsetMm = 450.0;
constexpr double kShotPoseYOffsetMm = 670.0;

constexpr ShotPose kYellowShotPoseA = { { kShotPoseXOffsetMm,                  kShotPoseYOffsetMm,                  180.0 }, -1.0 };
constexpr ShotPose kYellowShotPoseB = { { kFieldWidthMm - kShotPoseXOffsetMm,  kShotPoseYOffsetMm,                  180.0 },  1.0 };
constexpr ShotPose kBlueShotPoseA   = { { kShotPoseXOffsetMm,                  kFieldHeightMm - kShotPoseYOffsetMm, 180.0 },  1.0 };
constexpr ShotPose kBlueShotPoseB   = { { kFieldWidthMm - kShotPoseXOffsetMm,  kFieldHeightMm - kShotPoseYOffsetMm, 180.0 }, -1.0 };

class OffenseStateMachine
{
public:
  OffenseStateMachine(
    CompassSensor& compassSensor,
    Calibration& calibration,
    LinePCBComm& linePCBComm,
    Cam& camera,
    Orbit& orbit,
    Movement& movement,
    ModeControl& modeControl);

  void run(OffenseState configuredState);

  double lineAngle() const { return _lineAngle; }
  double avoidanceAngle() const { return _avoidanceAngle; }

private:
  CompassSensor& _compassSensor;
  Calibration& _calibration;
  LinePCBComm& _linePCBComm;
  Cam& _camera;
  Orbit& _orbit;
  Movement& _movement;
  ModeControl& _modeControl;

  double _lineAngle = -5;
  double _orbitAngle = -5;
  double _goalAngle = -5;
  double _avoidanceAngle = -5;
  bool _aimingGoal = false;

  // Runtime state for the PathPlan -> DribblerToKick -> SpinShot -> Kick
  // shooting sequence.
  OffenseState _activeState = OffenseState::PathPlan;
  // Shot pose locked in when PathPlan begins so the target does not flip
  // mid-drive. Cleared whenever the sequence is reset.
  const ShotPose* _targetShotPose = nullptr;
  // Times how long DribblerToKick has held the dribbler at full speed.
  elapsedMillis _dribblerToKickTimer;

  void updateVisionAndLineState();
  void runLineAvoidance();
  void runOrbitState();

  void runShootSequence();
  void runPathPlanState();
  void runDribblerToKickState();
  void runSpinShotState();
  void runKickState();
  const ShotPose& selectNearestShotPose() const;
  void resetSequence();

  void printDebugState() const;
};

#endif
