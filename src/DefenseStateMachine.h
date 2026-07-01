#ifndef DEFENSE_STATE_MACHINE_H
#define DEFENSE_STATE_MACHINE_H

class Defense;
class Movement;
class ModeControl;
class CompassSensor;
class GoalieCurveBoundary;
class VirtualBoundary;
class GameState;

// ---- Defense bad-zone tuning (kept here so they are easy to find/tune) ----
// If the line normal would push the robot toward a wall corner (field-relative
// heading magnitude over kBadZoneHeadingLimitDeg) while it is trying to slide that
// way, stop instead. kBadZoneMoveToleranceDeg is how close (deg) the move angle must
// be to the slide direction to count as "sliding that way".
constexpr double kBadZoneHeadingLimitDeg  = 53.0;
constexpr double kBadZoneMoveToleranceDeg = 30.0;

// Orchestrates the defensive behavior, mirroring OffenseStateMachine. It uses the
// Defense class as its pure-math collaborator (as OffenseStateMachine uses Orbit).
// Per-loop world state is read from gameState; the main loop owns gameState.update().
class DefenseStateMachine
{
public:
  DefenseStateMachine(Defense& defense,
                      Movement& movement,
                      ModeControl& modeControl,
                      CompassSensor& compassSensor,
                      GoalieCurveBoundary& goalieCurveBoundary,
                      VirtualBoundary& virtualBoundary);

  void run(GameState& gameState);

private:
  Defense&             _defense;
  Movement&            _movement;
  ModeControl&         _modeControl;
  CompassSensor&       _compassSensor;
  GoalieCurveBoundary& _goalieCurveBoundary;
  VirtualBoundary&     _virtualBoundary;

  bool shouldUseGoalieCurveBoundary(double lineAngle) const;
};

#endif
