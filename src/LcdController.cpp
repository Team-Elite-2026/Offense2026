#include <LcdController.h>
#include <string.h>

LcdController::LcdController(HardwareSerial& serial, LinePCBComm& linePCBComm,
                               CompassSensor& compassSensor, Switch& switches,
                               Movement& movement, RobotMode& robotMode)
  : _serial(serial),
    _linePCBComm(linePCBComm),
    _compassSensor(compassSensor),
    _switches(switches),
    _movement(movement),
    _robotMode(robotMode),
    _commandLength(0)
{
  state = {false, false, false, true, false, false, false,
           LcdStartMode::None, LcdStartPosition::None, 0, 0};
}

void LcdController::begin(uint32_t baud)
{
  _serial.begin(baud);
}

void LcdController::applyRobotModeSettings()
{
  if (_robotMode == RobotMode::Offense)
  {
    _movement.myPID->SetTunings(0.3, _movement.ki, _movement.kd);
  }
}

bool LcdController::isStartEnabled() const
{
  if (state.startOverrideActive)
  {
    return state.startEnabled;
  }
  return _switches.start();
}

bool LcdController::isGoalBlueSelected() const
{
  if (state.goalOverrideActive)
  {
    return state.goalIsBlue;
  }
  return _switches.goalSide();
}

uint8_t LcdController::telemetryModeOverride() const
{
  if (!state.robotModeOverrideActive)
  {
    return 0u;
  }
  return (_robotMode == RobotMode::Offense) ? 1u : 2u;
}

void LcdController::printLine(const String& line)
{
  _serial.println(line);
}

void LcdController::sendLineArray()
{
  const int16_t* vals = _linePCBComm.getActivatedVals();
  _serial.print("LACT:");
  for (int i = 0; i < 48; i++)
  {
    _serial.print(vals[i]);
    if (i < 47)
    {
      _serial.print(",");
    }
  }
  _serial.println();
}

void LcdController::sendTelemetry(double lineAngle, double avoidanceAngle)
{
  unsigned long now = millis();
  if (now - state.lastTelemetryMs < kTelemetryIntervalMs)
  {
    return;
  }
  state.lastTelemetryMs = now;

  printLine(isGoalBlueSelected() ? "blue goal" : "yellow goal");
  printLine("Mode: " + String(robotModeToken(_robotMode)));
  printLine(String("Light Gate: ") + (_switches.lightgate() ? "BLOCKED" : "CLEAR"));
  printLine("Battery: -1");
  printLine(state.lineCalibrationActive ? "Calibrating" : "Line Cal: IDLE");
  printLine("Orientation angle" + String(_compassSensor.getOrientation()));
  printLine("Line Angle: " + String(lineAngle));
  printLine("Avoidance angle: " + String(avoidanceAngle));
  sendLineArray();
}

void LcdController::sendCalibrationStatus()
{
  unsigned long now = millis();
  if (now - state.lastCalibrationStatusMs < kCalibrationStatusIntervalMs)
  {
    return;
  }
  state.lastCalibrationStatusMs = now;
  printLine("Calibrating");
}

bool LcdController::handleStartPositionCommand(const char* command)
{
  if (strncmp(command, "STARTPOS:", 9) != 0)
  {
    return false;
  }

  const char* modeToken = command + 9;
  const char* separator = strchr(modeToken, ':');
  if (separator == NULL)
  {
    return true;
  }

  char modeBuffer[16];
  size_t modeLength = (size_t)(separator - modeToken);
  if (modeLength == 0 || modeLength >= sizeof(modeBuffer))
  {
    return true;
  }

  memcpy(modeBuffer, modeToken, modeLength);
  modeBuffer[modeLength] = '\0';

  LcdStartMode parsedMode = LcdStartMode::None;
  LcdStartPosition parsedPosition = LcdStartPosition::None;
  if (!parseStartMode(modeBuffer, parsedMode) || !parseStartPosition(separator + 1, parsedPosition))
  {
    return true;
  }

  state.hasStartPosition = true;
  state.startMode        = parsedMode;
  state.startPosition    = parsedPosition;

  Serial.print("LCD Start Position: ");
  Serial.print(startModeToken(state.startMode));
  Serial.print(" / ");
  Serial.println(startPositionToken(state.startPosition));

  return true;
}

void LcdController::handleCommand(const char* command)
{
  if (command[0] == '\0')
  {
    return;
  }

  Serial.print("LCD CMD: ");
  Serial.println(command);

  if (strcmp(command, "CMD:START") == 0)
  {
    state.startOverrideActive = true;
    state.startEnabled        = true;
    return;
  }

  if (strcmp(command, "CMD:STOP") == 0)
  {
    state.startOverrideActive = true;
    state.startEnabled        = false;
    _movement.stop();
    return;
  }

  if (strcmp(command, "GOAL:BLUE") == 0)
  {
    state.goalOverrideActive = true;
    state.goalIsBlue         = true;
    return;
  }

  if (strcmp(command, "GOAL:YELLOW") == 0)
  {
    state.goalOverrideActive = true;
    state.goalIsBlue         = false;
    return;
  }

  if (strcmp(command, "MODE:OFFENSE") == 0)
  {
    state.robotModeOverrideActive = true;
    _robotMode = RobotMode::Offense;
    applyRobotModeSettings();
    return;
  }

  if (strcmp(command, "MODE:DEFENSE") == 0)
  {
    state.robotModeOverrideActive = true;
    _robotMode = RobotMode::Defense;
    applyRobotModeSettings();
    return;
  }

  if (strcmp(command, "MODE:AUTO") == 0)
  {
    state.robotModeOverrideActive = false;
    return;
  }

  if (strcmp(command, "CMD:CALIB_LINE_START") == 0)
  {
    state.lineCalibrationActive = true;
    _movement.stop();
    return;
  }

  if (strcmp(command, "CMD:CALIB_LINE_STOP") == 0)
  {
    state.lineCalibrationActive = false;
    return;
  }

  handleStartPositionCommand(command);
}

void LcdController::readCommands()
{
  while (_serial.available() > 0)
  {
    char incoming = (char)_serial.read();
    if (incoming == '\r')
    {
      continue;
    }

    if (incoming == '\n')
    {
      _commandBuffer[_commandLength] = '\0';
      handleCommand(_commandBuffer);
      _commandLength = 0;
      continue;
    }

    if (_commandLength < (kCommandBufferSize - 1))
    {
      _commandBuffer[_commandLength++] = incoming;
    }
  }
}

const char* LcdController::robotModeToken(RobotMode mode)
{
  return (mode == RobotMode::Offense) ? "OFFENSE" : "DEFENSE";
}

const char* LcdController::startModeToken(LcdStartMode mode)
{
  switch (mode)
  {
    case LcdStartMode::Offense: return "OFFENSE";
    case LcdStartMode::Defense: return "DEFENSE";
    default:                    return "NONE";
  }
}

const char* LcdController::startPositionToken(LcdStartPosition position)
{
  switch (position)
  {
    case LcdStartPosition::BehindBall:        return "BEHIND_BALL";
    case LcdStartPosition::BehindCenterRing:  return "BEHIND_CENTER_RING";
    case LcdStartPosition::Goal:              return "GOAL";
    case LcdStartPosition::Neutral1:          return "NEUTRAL_1";
    case LcdStartPosition::Neutral2:          return "NEUTRAL_2";
    case LcdStartPosition::Neutral3:          return "NEUTRAL_3";
    case LcdStartPosition::Neutral4:          return "NEUTRAL_4";
    default:                                  return "NONE";
  }
}

bool LcdController::parseStartMode(const char* token, LcdStartMode& mode)
{
  if (strcmp(token, "OFFENSE") == 0) { mode = LcdStartMode::Offense; return true; }
  if (strcmp(token, "DEFENSE") == 0) { mode = LcdStartMode::Defense; return true; }
  return false;
}

bool LcdController::parseStartPosition(const char* token, LcdStartPosition& position)
{
  if (strcmp(token, "BEHIND_BALL") == 0)        { position = LcdStartPosition::BehindBall;       return true; }
  if (strcmp(token, "BEHIND_CENTER_RING") == 0) { position = LcdStartPosition::BehindCenterRing; return true; }
  if (strcmp(token, "GOAL") == 0)               { position = LcdStartPosition::Goal;             return true; }
  if (strcmp(token, "NEUTRAL_1") == 0)          { position = LcdStartPosition::Neutral1;         return true; }
  if (strcmp(token, "NEUTRAL_2") == 0)          { position = LcdStartPosition::Neutral2;         return true; }
  if (strcmp(token, "NEUTRAL_3") == 0)          { position = LcdStartPosition::Neutral3;         return true; }
  if (strcmp(token, "NEUTRAL_4") == 0)          { position = LcdStartPosition::Neutral4;         return true; }
  return false;
}
