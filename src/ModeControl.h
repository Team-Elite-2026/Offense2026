#ifndef MODE_CONTROL_H
#define MODE_CONTROL_H

#include <Arduino.h>
#include <LinePCBComm.h>
#include <CompassSensor.h>
#include <Movement.h>

enum class RobotMode { Offense, Defense };
enum class StartMode { None, Offense, Defense };
enum class StartPosition
{
  None,
  BehindBall,
  BehindCenterRing,
  Goal,
  Neutral1,
  Neutral2,
  Neutral3,
  Neutral4
};

struct ModeControlState
{
  bool goalIsBlue;
  bool robotModeOverrideActive;
  bool lineCalibrationActive;
  bool compassCalibrationRequested;
  bool hasStartPosition;
  StartMode startMode;
  StartPosition startPosition;
  unsigned long lastTelemetryMs;
  unsigned long lastCalibrationStatusMs;
};

class ModeControl
{
public:
  ModeControl(HardwareSerial& serial, LinePCBComm& linePCBComm,
              CompassSensor& compassSensor, Movement& movement,
              RobotMode& robotMode);

  void begin(uint32_t baud);
  void readCommands();
  void sendTelemetry(double lineAngle, double avoidanceAngle);
  void sendCalibrationStatus();
  void applyRobotModeSettings();

  bool isStartEnabled() const;
  bool isGoalBlueSelected() const;
  uint8_t telemetryModeOverride() const;

  ModeControlState state;

private:
  static constexpr uint8_t       kStartPin                    = 38;
  static constexpr uint8_t       kLightGatePin                = 41;
  static constexpr unsigned long kTelemetryIntervalMs         = 250;
  static constexpr unsigned long kCalibrationStatusIntervalMs = 250;
  static constexpr size_t        kCommandBufferSize           = 96;

  HardwareSerial& _serial;
  LinePCBComm&    _linePCBComm;
  CompassSensor&  _compassSensor;
  Movement&       _movement;
  RobotMode&      _robotMode;

  char   _commandBuffer[kCommandBufferSize];
  size_t _commandLength;

  void printLine(const String& line);
  void sendLineArray();
  void handleCommand(const char* command);
  bool handleStartPositionCommand(const char* command);

  static const char* robotModeToken(RobotMode mode);
  static const char* startModeToken(StartMode mode);
  static const char* startPositionToken(StartPosition position);
  static bool        parseStartMode(const char* token, StartMode& mode);
  static bool        parseStartPosition(const char* token, StartPosition& position);
};

#endif
