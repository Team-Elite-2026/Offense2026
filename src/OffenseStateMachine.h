#ifndef OFFENSE_STATE_MACHINE_H
#define OFFENSE_STATE_MACHINE_H

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
  SpinShot,
};

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

  void run(OffenseState configuredState, const Point& spinShotTargetPose);

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

  void updateVisionAndLineState();
  void runLineAvoidance();
  void runOrbitState();
  void runSpinShotState(const Point& spinShotTargetPose);
  void printDebugState() const;
};

#endif
