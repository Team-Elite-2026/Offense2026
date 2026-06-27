#include <ModeControl.h>
#include <string.h>

ModeControl::ModeControl(HardwareSerial& serial, LinePCBComm& linePCBComm,
                         CompassSensor& compassSensor, Movement& movement,
                         RobotMode& robotMode)
  : _serial(serial),
    _linePCBComm(linePCBComm),
    _compassSensor(compassSensor),
    _movement(movement),
    _robotMode(robotMode),
    _commandLength(0),
    _batteryVoltage(0.0f)
{
  state = {true, false, false, false, false, false,
           StartMode::None, StartPosition::None, 0};
}

void ModeControl::begin(uint32_t baud)
{
  _serial.begin(baud);
  pinMode(kStartPin, INPUT);
  pinMode(kLightGatePin, INPUT_PULLUP);
  pinMode(kBatterySensePin, INPUT);
  analogReadResolution(12);
  analogReadAveraging(kBatterySampleCount);
}

void ModeControl::applyRobotModeSettings()
{
  if (_robotMode == RobotMode::Offense)
  {
    _movement.myPID->SetTunings(0.3, _movement.ki, _movement.kd);
  }
}

void ModeControl::sendBootMarker()
{
  printLine("LCD:BOOT");
  _serial.flush();
}

Print* ModeControl::statusOutput()
{
  return &_serial;
}

bool ModeControl::isStartEnabled() const
{
  return digitalRead(kStartPin) == HIGH;
}

bool ModeControl::isGoalBlueSelected() const
{
  return state.goalIsBlue;
}

bool ModeControl::doWeHaveBall() const 
{
  bool lightGateBlocked = digitalRead(kLightGatePin) == LOW;
  return lightGateBlocked;

}


uint8_t ModeControl::telemetryModeOverride() const
{
  if (!state.robotModeOverrideActive)
  {
    return 0u;
  }
  return (_robotMode == RobotMode::Offense) ? 1u : 2u;
}

void ModeControl::printLine(const String& line)
{
  _serial.println(line);
}

void ModeControl::sendLineArray()
{
  const int16_t* vals = _linePCBComm.getActivatedVals();
  _serial.print("LACT:");
  for (int i = 0; i < 48; i++)
  {
    _serial.print(vals[i] != 0 ? 1 : 0);
    if (i < 47)
    {
      _serial.print(",");
    }
  }
  _serial.println();
}

float ModeControl::readBatteryVoltage()
{
  uint32_t rawTotal = 0;
  for (uint8_t sample = 0; sample < kBatterySampleCount; sample++)
  {
    rawTotal += analogRead(kBatterySensePin);
  }

  const float rawAverage = (float)rawTotal / kBatterySampleCount;
  const float measuredVoltage = rawAverage * (kAdcReferenceVolts / kAdcMaxValue) *
                                kBatteryDividerScale;

  if (_batteryVoltage <= 0.0f)
  {
    _batteryVoltage = measuredVoltage;
  }
  else
  {
    _batteryVoltage = (_batteryVoltage * 0.75f) + (measuredVoltage * 0.25f);
  }

  return _batteryVoltage;
}

void ModeControl::sendCalibrationStatus()
{
  unsigned long now = millis();
  if (now - state.lastCalibrationStatusMs < kCalibrationStatusIntervalMs)
  {
    return;
  }
  state.lastCalibrationStatusMs = now;
  printLine("Calibrating");
}

bool ModeControl::handleStartPositionCommand(const char* command)
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

  StartMode parsedMode = StartMode::None;
  StartPosition parsedPosition = StartPosition::None;
  if (!parseStartMode(modeBuffer, parsedMode) || !parseStartPosition(separator + 1, parsedPosition))
  {
    return true;
  }

  state.hasStartPosition = true;
  state.startMode        = parsedMode;
  state.startPosition    = parsedPosition;

  Serial.print("Start Position: ");
  Serial.print(startModeToken(state.startMode));
  Serial.print(" / ");
  Serial.println(startPositionToken(state.startPosition));

  return true;
}

void ModeControl::handleCommand(const char* command)
{
  if (command[0] == '\0')
  {
    return;
  }

  Serial.print("CMD: ");
  Serial.println(command);

  if (strcmp(command, "GOAL:BLUE") == 0)
  {
    state.goalIsBlue = true;
    return;
  }

  if (strcmp(command, "GOAL:YELLOW") == 0)
  {
    state.goalIsBlue = false;
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
    _linePCBComm.triggerCalibration();
    return;
  }

  if (strcmp(command, "CMD:CALIB_LINE_STOP") == 0)
  {
    state.lineCalibrationActive = false;
    _linePCBComm.endCalibration();
    return;
  }

  if (strcmp(command, "CMD:LINE_DEBUG_ON") == 0)
  {
    if (!state.lineDebugEnabled)
    {
      state.lineDebugEnabled = true;
      _linePCBComm.setDebugEnabled(true);
    }
    return;
  }

  if (strcmp(command, "CMD:LINE_DEBUG_OFF") == 0)
  {
    if (state.lineDebugEnabled)
    {
      state.lineDebugEnabled = false;
      _linePCBComm.setDebugEnabled(false);
    }
    return;
  }

  handleStartPositionCommand(command);
}

void ModeControl::readCommands()
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

const char* ModeControl::robotModeToken(RobotMode mode)
{
  return (mode == RobotMode::Offense) ? "OFFENSE" : "DEFENSE";
}

const char* ModeControl::startModeToken(StartMode mode)
{
  switch (mode)
  {
    case StartMode::Offense: return "OFFENSE";
    case StartMode::Defense: return "DEFENSE";
    default:                 return "NONE";
  }
}

const char* ModeControl::startPositionToken(StartPosition position)
{
  switch (position)
  {
    case StartPosition::BehindBall:        return "BEHIND_BALL";
    case StartPosition::BehindCenterRing:  return "BEHIND_CENTER_RING";
    case StartPosition::Goal:              return "GOAL";
    case StartPosition::Neutral1:          return "NEUTRAL_1";
    case StartPosition::Neutral2:          return "NEUTRAL_2";
    case StartPosition::Neutral3:          return "NEUTRAL_3";
    case StartPosition::Neutral4:          return "NEUTRAL_4";
    default:                               return "NONE";
  }
}

bool ModeControl::parseStartMode(const char* token, StartMode& mode)
{
  if (strcmp(token, "OFFENSE") == 0) { mode = StartMode::Offense; return true; }
  if (strcmp(token, "DEFENSE") == 0) { mode = StartMode::Defense; return true; }
  return false;
}

bool ModeControl::parseStartPosition(const char* token, StartPosition& position)
{
  if (strcmp(token, "BEHIND_BALL") == 0)        { position = StartPosition::BehindBall;       return true; }
  if (strcmp(token, "BEHIND_CENTER_RING") == 0) { position = StartPosition::BehindCenterRing; return true; }
  if (strcmp(token, "GOAL") == 0)               { position = StartPosition::Goal;             return true; }
  if (strcmp(token, "NEUTRAL_1") == 0)          { position = StartPosition::Neutral1;         return true; }
  if (strcmp(token, "NEUTRAL_2") == 0)          { position = StartPosition::Neutral2;         return true; }
  if (strcmp(token, "NEUTRAL_3") == 0)          { position = StartPosition::Neutral3;         return true; }
  if (strcmp(token, "NEUTRAL_4") == 0)          { position = StartPosition::Neutral4;         return true; }
  return false;
}
