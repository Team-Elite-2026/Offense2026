#include <ModeControl.h>
#include <string.h>
#include <stdlib.h>

ModeControl::ModeControl(HardwareSerial& serial, LinePCBComm& linePCBComm,
                         CompassSensor& compassSensor, Movement& movement)
  : _serial(serial),
    _linePCBComm(linePCBComm),
    _compassSensor(compassSensor),
    _movement(movement),
    _commandLength(0),
    _batteryVoltage(0.0f),
    _nextDebugKickMs(kDebugKickCooldownMs)
{
  state = {true, false, false, false, false, false,
           StartMode::None, StartPosition::None, 0, 0, defaultRobotMode};
}

void ModeControl::begin(uint32_t baud)
{
  _serial.begin(baud);
  pinMode(kStartPin, INPUT);
  pinMode(kLightGatePin, INPUT_PULLUP);
  pinMode(kDebugKickerPin, OUTPUT);
  digitalWrite(kDebugKickerPin, LOW);
  pinMode(kDebugDribblerPinA, OUTPUT);
  pinMode(kDebugDribblerPinB, OUTPUT);
  pinMode(kDebugDribblerPwmPin, OUTPUT);
  setDebugDribbler(0, 0);
  pinMode(kBatterySensePin, INPUT);
  _nextDebugKickMs = millis() + kDebugKickCooldownMs;
  analogReadResolution(12);
  analogReadAveraging(kBatterySampleCount);
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
  return digitalRead(kStartPin) == HIGH && digitalRead(commPin) == HIGH;
}

bool ModeControl::isGoalBlueSelected() const
{
  return state.goalIsBlue;
}

bool ModeControl::isOffenseMode() const {
  return state.robotMode == RobotMode::Offense;
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
  return (state.robotMode == RobotMode::Offense) ? 1u : 2u;
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

void ModeControl::sendTelemetry(double lineAngle, double avoidanceAngle, double poseX, double poseY)
{
  unsigned long now = millis();
  if (now - state.lastTelemetryMs < kTelemetryIntervalMs)
  {
    return;
  }
  state.lastTelemetryMs = now;

  printLine(isGoalBlueSelected() ? "blue goal" : "yellow goal");
  printLine(String("Mode: ") + (state.robotModeOverrideActive ? robotModeToken(state.robotMode) : "AUTO"));
  bool lightGateBlocked = digitalRead(kLightGatePin) == LOW;
  printLine(String("Light Gate: ") + (lightGateBlocked ? "BLOCKED" : "CLEAR"));
  printLine(String("Battery: ") + String(readBatteryVoltage(), 1));
  printLine(state.lineCalibrationActive ? "Calibrating" : "Line Cal: IDLE");
  printLine("Orientation angle: " + String(_compassSensor.getOrientation()));
  printLine("Line Angle: " + String(lineAngle));
  printLine("Avoidance angle: " + String(avoidanceAngle));
  printLine("Pose X: " + String(poseX, 0));
  printLine("Pose Y: " + String(poseY, 0));
  if (state.lineDebugEnabled)
  {
    sendLineArray();
  }
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

void ModeControl::debugKick()
{
  unsigned long now = millis();
  if (now < _nextDebugKickMs)
  {
    printLine("Kick: WAIT");
    return;
  }

  digitalWrite(kDebugKickerPin, HIGH);
  delay(10);
  digitalWrite(kDebugKickerPin, LOW);
  _nextDebugKickMs = millis() + kDebugKickCooldownMs;
  printLine("Kick: DONE");
}

void ModeControl::setDebugDribbler(int8_t direction, uint8_t pwm)
{
  const char* directionText = "STOP ";
  if (direction > 0 && pwm > 0)
  {
    digitalWrite(kDebugDribblerPinA, HIGH);
    digitalWrite(kDebugDribblerPinB, LOW);
    directionText = "FWD ";
  }
  else if (direction < 0 && pwm > 0)
  {
    digitalWrite(kDebugDribblerPinA, LOW);
    digitalWrite(kDebugDribblerPinB, HIGH);
    directionText = "BACK ";
  }
  else
  {
    digitalWrite(kDebugDribblerPinA, LOW);
    digitalWrite(kDebugDribblerPinB, LOW);
    pwm = 0;
  }

  analogWrite(kDebugDribblerPwmPin, pwm);
  printLine(String("Dribbler: ") + directionText + String(pwm));
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
    state.robotMode = RobotMode::Offense;
    return;
  }

  if (strcmp(command, "MODE:DEFENSE") == 0)
  {
    state.robotModeOverrideActive = true;
    state.robotMode = RobotMode::Defense;
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

  if (strcmp(command, "CMD:KICK") == 0)
  {
    debugKick();
    return;
  }

  if (strcmp(command, "CMD:DRIBBLER:STOP") == 0)
  {
    setDebugDribbler(0, 0);
    return;
  }

  if (strncmp(command, "CMD:DRIBBLER:FWD:", 17) == 0)
  {
    int pwm = atoi(command + 17);
    pwm = constrain(pwm, 0, 255);
    setDebugDribbler(1, (uint8_t)pwm);
    return;
  }

  if (strncmp(command, "CMD:DRIBBLER:BACK:", 18) == 0)
  {
    int pwm = atoi(command + 18);
    pwm = constrain(pwm, 0, 255);
    setDebugDribbler(-1, (uint8_t)pwm);
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
