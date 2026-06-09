#ifndef LCD_CONTROLLER_H
#define LCD_CONTROLLER_H

#include <Arduino.h>
#include <LinePCBComm.h>
#include <CompassSensor.h>
#include <Switches.h>
#include <Movement.h>

enum class RobotMode { Offense, Defense };
enum class LcdStartMode { None, Offense, Defense };
enum class LcdStartPosition
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

struct LcdControlState
{
  bool startOverrideActive;
  bool startEnabled;
  bool goalOverrideActive;
  bool goalIsBlue;
  bool robotModeOverrideActive;
  bool lineCalibrationActive;
  bool hasStartPosition;
  LcdStartMode startMode;
  LcdStartPosition startPosition;
  unsigned long lastTelemetryMs;
  unsigned long lastCalibrationStatusMs;
};

class LcdController
{
public:
  LcdController(HardwareSerial& serial, LinePCBComm& linePCBComm,
                CompassSensor& compassSensor, Switch& switches,
                Movement& movement, RobotMode& robotMode);

  void begin(uint32_t baud);
  void readCommands();
  void sendTelemetry(double lineAngle, double avoidanceAngle);
  void sendCalibrationStatus();
  void applyRobotModeSettings();

  bool isStartEnabled() const;
  bool isGoalBlueSelected() const;
  uint8_t telemetryModeOverride() const;

  LcdControlState state;

private:
  static constexpr unsigned long kTelemetryIntervalMs       = 250;
  static constexpr unsigned long kCalibrationStatusIntervalMs = 250;
  static constexpr size_t        kCommandBufferSize          = 96;

  HardwareSerial& _serial;
  LinePCBComm&    _linePCBComm;
  CompassSensor&  _compassSensor;
  Switch&         _switches;
  Movement&       _movement;
  RobotMode&      _robotMode;

  char   _commandBuffer[kCommandBufferSize];
  size_t _commandLength;

  void printLine(const String& line);
  void sendLineArray();
  void handleCommand(const char* command);
  bool handleStartPositionCommand(const char* command);

  static const char* robotModeToken(RobotMode mode);
  static const char* startModeToken(LcdStartMode mode);
  static const char* startPositionToken(LcdStartPosition position);
  static bool        parseStartMode(const char* token, LcdStartMode& mode);
  static bool        parseStartPosition(const char* token, LcdStartPosition& position);
};

#endif
