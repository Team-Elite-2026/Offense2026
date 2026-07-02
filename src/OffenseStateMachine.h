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

// Field frame matches the pose the Pi sends: origin (0, 0) at the field center,
// +y toward the attacked goal (the goal we face at zero heading), +x to the
// robot's right at zero heading, in mm.
constexpr double kFieldWidthMm  = 1820.0;
constexpr double kFieldHeightMm = 2430.0;

// A shooting pose plus the spin direction to use once the robot arrives.
// spinSign is -1 (spin left) or +1 (spin right); used by SpinShot's search.
struct ShotPose
{
  Point pose;
  double spinSign;
};

// Because the frame is always attack-relative (+y is the attacked goal), there is
// a single pair of shot poses regardless of which physical goal we attack.  They
// sit 450mm in x inward from each side wall and 670mm in y inward from the
// attacked-goal end:
//   x = +/-(kFieldWidthMm/2 - 450) = +/-460,  y = kFieldHeightMm/2 - 670 = +545.
// Both face heading 180 (kicker toward the attacked goal once the robot spins).
constexpr double kShotPoseXOffsetMm = 450.0;
constexpr double kShotPoseYOffsetMm = 670.0;

constexpr double kShotPoseX = kFieldWidthMm * 0.5 - kShotPoseXOffsetMm;   //  460
constexpr double kShotPoseY = kFieldHeightMm * 0.5 - kShotPoseYOffsetMm;  //  545

// Right-side pose (+x) sweeps left, left-side pose (-x) sweeps right, so each
// spins the ball toward the centre of the attacked goal.
constexpr ShotPose kShotPoseRight = { {  kShotPoseX, kShotPoseY, 180.0 },  1.0 };
constexpr ShotPose kShotPoseLeft  = { { -kShotPoseX, kShotPoseY, 180.0 }, -1.0 };

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

  // Reads camera data and snapshots the latest LinePCBComm values. The main
  // loop owns LinePCBComm::update() so movement and telemetry stay in sync.
  void updateVisionAndLineState();


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
  // Times continuous ball loss in Orbit before returning to field center.
  elapsedMillis _orbitLostBallTimer;
  bool _orbitLostBallTimerActive = false;

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
