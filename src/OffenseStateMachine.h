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

// PathPlan: how close (mm) to the shot pose counts as "arrived".
constexpr double kArrivalMarginMm = 30.0;

// SpinShot: while the goal is not yet in view we sweep at a fixed speed in the
// chosen pose's search direction; once it is in view we close on it with a PID
// whose spin speed scales with the heading error to the goal.
constexpr double kSpinSearchSpeed    = 0.06;
constexpr double kSpinShotMaxSpeed   = 0.18;
constexpr double kGoalAlignedDegrees = 12.0;

// Orbit ball approach: when the ball is closer than this (cm) slow down and
// start the dribbler to draw the ball in.
constexpr double kBallCloseCm       = 15.0;
constexpr double kBallApproachSpeed = 0.2;

// Orbit deceleration: ease from full offense speed down to a capture-speed floor
// as the robot closes on the behind-the-ball target point.
constexpr double kOrbitDecelRangeCm = 35.0;
constexpr double kMinOrbitCaptureSpeed = 0.18;

// When the ball is dead ahead, push/shoot at full speed instead of applying the
// orbit deceleration ramp.
constexpr double kOrbitForwardDeadbandDeg = 5.0;

// Orbit lost-ball recovery: wait this long before returning to field center.
constexpr unsigned long kOrbitLostBallCenterDelayMs = 1000;
constexpr double kOrbitLostBallRecoveryX = 0.0;
constexpr double kOrbitLostBallRecoveryY = 0.0;
constexpr double kOrbitLostBallRecoveryHeading = 0.0;

// Orbit + line avoidance: keep orbit motion parallel to the line while adding
// enough outward normal to get off the boundary.
constexpr double kOrbitLineAvoidanceSpeed = 0.4;
constexpr double kOrbitLineMinOutwardGain = 0.35;
constexpr double kOrbitLineChordOutwardGain = 0.65;
constexpr double kOrbitLineDefaultChordLength = 0.5;
constexpr double kOrbitLineMinVectorMagnitudeSq = 1e-6;

// Goal bearing fallback used when the camera does not currently see the goal.
constexpr double kFallbackGoalX = 0.0;
constexpr double kFallbackGoalY = 1075.0;
constexpr double kFallbackGoalHeading = 0.0;

// Dribbler PWM setpoints (0..255), converted to motor speed factors.
constexpr double kPwmMax = 255.0;
constexpr double kDribblerApproachPwm = 96.0;
constexpr double kDribblerTravelPwm   = 145.0;
constexpr double kDribblerMaxPwm      = 255.0;

// DribblerToKick: hold the dribbler at full speed this long before spinning.
constexpr unsigned long kDribblerSpinUpMs = 250;

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
