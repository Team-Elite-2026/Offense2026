#ifndef GAME_STATE_H
#define GAME_STATE_H

#include <trig.h>

class Cam;
class LinePCBComm;
class CompassSensor;
class ModeControl;
class Movement;

// Single source of truth for per-loop world state. update() snapshots every sensor
// value once per loop; the resolved goal accessors fold in the camera-vs-pose
// fallback so that logic lives in exactly one place.
class GameState
{
public:
  GameState(Cam& camera,
            LinePCBComm& linePCBComm,
            CompassSensor& compassSensor,
            ModeControl& modeControl,
            Movement& movement);

  // Snapshots all sensor state. Call once per loop, after LinePCBComm::update()
  // and before dispatching to the offense/defense state machines.
  void update();

  // ---- raw snapshots (filled by update) ----
  double ballAngle = -5;          // raw camera ball angle (0..360), -5 if unseen
  double ballDistance = -5;       // cm, -5 if unseen
  double ballDerivative = -5;
  int    ballSampleTime = 0;
  double predictedBallAngle = -5;

  double lineAngle = -5;          // -5 if no line under the robot
  double avoidanceAngle = -5;
  double chordLength = -5;
  bool   crossLine = false;

  Point  pose;                    // x,y (mm) from LIDAR; heading = compass offset
  bool   hasPose = false;
  double heading = 0;             // compass offset, degrees

  bool   goalIsBlue = false;
  double cameraAttackGoal = -5;   // camera angle to the goal we attack, -5 if unseen
  double cameraHomeGoal   = -5;   // camera angle to the goal we defend, -5 if unseen

  // ---- resolved goal accessors ----
  // Camera angle when the goal is seen, else a pose-derived angle from the LIDAR
  // localization, else -5. Returned in camera convention (normalize180: 0 = ahead).
  double attackGoalAngle() const;
  bool   hasAttackGoal() const;
  double homeGoalAngle() const;
  bool   hasHomeGoal() const;

  // Predicted-or-raw ball angle used by defense (Cam::selectedDefenseBallAngle).
  double defenseBallAngle() const;

private:
  Cam&           _camera;
  LinePCBComm&   _linePCBComm;
  CompassSensor& _compassSensor;
  ModeControl&   _modeControl;
  Movement&      _movement;

  // Robot-relative angle to a goal centred at (0, goalCenterY) in the pose frame,
  // normalize180. Returns -5 when there is no valid pose.
  double poseGoalAngle(double goalCenterY) const;
};

#endif
