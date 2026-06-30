#ifndef DEFENSE_STATE_MACHINE_H
#define DEFENSE_STATE_MACHINE_H

class Defense;
class Movement;
class ModeControl;
class CompassSensor;
class GoalieCurveBoundary;
class VirtualBoundary;
class GameState;

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
