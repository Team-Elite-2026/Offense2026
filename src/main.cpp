#include <Arduino.h>
#include <LineDetection.h>
#include <CompassSensor.h>
#include <Switches.h>
#include <Callibration.h>
#include <Movement.h>
#include <orbit.h>
#include <Cam.h>
#include <Defense.h>
#include <trig.h>
#include <string.h>
#include <TrajectoryExecutor.h>

double pincontrolRLA = 22;
double pincontrolRLB = 23;
double pinspeedRL = 2;
double pincontrolRRA = 20;
double pincontrolRRB = 21;
double pinspeedRR = 3;
double pincontrolFRA = 18;
double pincontrolFRB = 19;
double pinspeedFR = 4;
double pincontrolFLA = 9;
double pincontrolFLB = 10;
double pinspeedFL = 5;

LineDetection lineDetection;
CompassSensor compassSensor;
Switch switches;
Calibration calibration(lineDetection, compassSensor);
Motor FL(pincontrolFLA, pincontrolFLB, pinspeedFL);
Motor FR(pincontrolFRA, pincontrolFRB, pinspeedFR);
Motor BL(pincontrolRLA, pincontrolRLB, pinspeedRL);
Motor BR(pincontrolRRA, pincontrolRRB, pinspeedRR);
Movement movement(FL, FR, BL, BR, compassSensor);
Orbit orbit(1);
Cam camera;
Defense defense;
TrajectoryExecutor trajectoryExecutor(FL, FR, BL, BR, compassSensor, switches);
HardwareSerial& lcdSerial = Serial8;

enum class RobotMode
{
  Offense,
  Defense
};

enum class LcdStartMode
{
  None,
  Offense,
  Defense
};

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
  bool lineCalibrationActive;
  bool hasStartPosition;
  LcdStartMode startMode;
  LcdStartPosition startPosition;
  unsigned long lastTelemetryMs;
  unsigned long lastCalibrationStatusMs;
};

RobotMode kRobotMode = RobotMode::Offense;
double defenseSpeedFactor = 0.26;
double offenseSpeedFactor = 0.22;
double lineAvoidanceSpeed = 0.15;
constexpr uint32_t kLcdBaud = 9600;
constexpr unsigned long kLcdTelemetryIntervalMs = 250;
constexpr unsigned long kLcdCalibrationStatusIntervalMs = 250;
constexpr size_t kLcdCommandBufferSize = 96;

LcdControlState lcdControl = {
  false,
  false,
  false,
  true,
  false,
  false,
  LcdStartMode::None,
  LcdStartPosition::None,
  0,
  0
};
char lcdCommandBuffer[kLcdCommandBufferSize];
size_t lcdCommandLength = 0;
double lineAngle, currentOffset, orbitAngle, maxChordLength, goalAngle, avoidanceAngle;
bool aimingGoal;

bool isStartEnabled()
{
  if (lcdControl.startOverrideActive)
  {
    return lcdControl.startEnabled;
  }
  return switches.start();
}

bool isGoalBlueSelected()
{
  if (lcdControl.goalOverrideActive)
  {
    return lcdControl.goalIsBlue;
  }
  return switches.goalSide();
}

const char* robotModeToken(RobotMode mode)
{
  return (mode == RobotMode::Offense) ? "OFFENSE" : "DEFENSE";
}

void applyRobotModeSettings()
{
  if (kRobotMode == RobotMode::Offense)
  {
    movement.myPID->SetTunings(0.3, movement.ki, movement.kd);
  }
}

void lcdPrintLine(const String& line)
{
  lcdSerial.println(line);
}

const char* lcdStartModeToken(LcdStartMode mode)
{
  switch (mode)
  {
    case LcdStartMode::Offense:
      return "OFFENSE";
    case LcdStartMode::Defense:
      return "DEFENSE";
    case LcdStartMode::None:
    default:
      return "NONE";
  }
}

const char* lcdStartPositionToken(LcdStartPosition position)
{
  switch (position)
  {
    case LcdStartPosition::BehindBall:
      return "BEHIND_BALL";
    case LcdStartPosition::BehindCenterRing:
      return "BEHIND_CENTER_RING";
    case LcdStartPosition::Goal:
      return "GOAL";
    case LcdStartPosition::Neutral1:
      return "NEUTRAL_1";
    case LcdStartPosition::Neutral2:
      return "NEUTRAL_2";
    case LcdStartPosition::Neutral3:
      return "NEUTRAL_3";
    case LcdStartPosition::Neutral4:
      return "NEUTRAL_4";
    case LcdStartPosition::None:
    default:
      return "NONE";
  }
}

bool parseLcdStartMode(const char* token, LcdStartMode& mode)
{
  if (strcmp(token, "OFFENSE") == 0)
  {
    mode = LcdStartMode::Offense;
    return true;
  }
  if (strcmp(token, "DEFENSE") == 0)
  {
    mode = LcdStartMode::Defense;
    return true;
  }
  return false;
}

bool parseLcdStartPosition(const char* token, LcdStartPosition& position)
{
  if (strcmp(token, "BEHIND_BALL") == 0)
  {
    position = LcdStartPosition::BehindBall;
    return true;
  }
  if (strcmp(token, "BEHIND_CENTER_RING") == 0)
  {
    position = LcdStartPosition::BehindCenterRing;
    return true;
  }
  if (strcmp(token, "GOAL") == 0)
  {
    position = LcdStartPosition::Goal;
    return true;
  }
  if (strcmp(token, "NEUTRAL_1") == 0)
  {
    position = LcdStartPosition::Neutral1;
    return true;
  }
  if (strcmp(token, "NEUTRAL_2") == 0)
  {
    position = LcdStartPosition::Neutral2;
    return true;
  }
  if (strcmp(token, "NEUTRAL_3") == 0)
  {
    position = LcdStartPosition::Neutral3;
    return true;
  }
  if (strcmp(token, "NEUTRAL_4") == 0)
  {
    position = LcdStartPosition::Neutral4;
    return true;
  }
  return false;
}

void logStartPositionSelection()
{
  Serial.print("LCD Start Position: ");
  Serial.print(lcdStartModeToken(lcdControl.startMode));
  Serial.print(" / ");
  Serial.println(lcdStartPositionToken(lcdControl.startPosition));
}

void sendLcdLineArray()
{
  lcdSerial.print("LACT:");
  for (int i = 0; i < 48; i++)
  {
    lcdSerial.print(lineDetection.activatedVals[i]);
    if (i < 47)
    {
      lcdSerial.print(",");
    }
  }
  lcdSerial.println();
}

void sendLcdTelemetry()
{
  unsigned long now = millis();
  if (now - lcdControl.lastTelemetryMs < kLcdTelemetryIntervalMs)
  {
    return;
  }
  lcdControl.lastTelemetryMs = now;

  if (isGoalBlueSelected())
  {
    lcdPrintLine("blue goal");
  }
  else
  {
    lcdPrintLine("yellow goal");
  }

  lcdPrintLine("Mode: " + String(robotModeToken(kRobotMode)));
  lcdPrintLine(String("Light Gate: ") + (switches.lightgate() ? "BLOCKED" : "CLEAR"));
  lcdPrintLine("Battery: -1");
  lcdPrintLine(lcdControl.lineCalibrationActive ? "Calibrating" : "Line Cal: IDLE");
  lcdPrintLine("Orientation angle" + String(compassSensor.getOrientation()));
  lcdPrintLine("Line Angle: " + String(lineAngle));
  lcdPrintLine("Avoidance angle: " + String(avoidanceAngle));
  sendLcdLineArray();
}

void sendLcdCalibrationStatus()
{
  unsigned long now = millis();
  if (now - lcdControl.lastCalibrationStatusMs < kLcdCalibrationStatusIntervalMs)
  {
    return;
  }
  lcdControl.lastCalibrationStatusMs = now;
  lcdPrintLine("Calibrating");
}

bool handleStartPositionCommand(const char* command)
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
  if (!parseLcdStartMode(modeBuffer, parsedMode) || !parseLcdStartPosition(separator + 1, parsedPosition))
  {
    return true;
  }

  lcdControl.hasStartPosition = true;
  lcdControl.startMode = parsedMode;
  lcdControl.startPosition = parsedPosition;
  logStartPositionSelection();
  return true;
}

void handleLcdCommand(const char* command)
{
  if (command[0] == '\0')
  {
    return;
  }

  Serial.print("LCD CMD: ");
  Serial.println(command);

  if (strcmp(command, "CMD:START") == 0)
  {
    lcdControl.startOverrideActive = true;
    lcdControl.startEnabled = true;
    return;
  }

  if (strcmp(command, "CMD:STOP") == 0)
  {
    lcdControl.startOverrideActive = true;
    lcdControl.startEnabled = false;
    movement.stop();
    return;
  }

  if (strcmp(command, "GOAL:BLUE") == 0)
  {
    lcdControl.goalOverrideActive = true;
    lcdControl.goalIsBlue = true;
    return;
  }

  if (strcmp(command, "GOAL:YELLOW") == 0)
  {
    lcdControl.goalOverrideActive = true;
    lcdControl.goalIsBlue = false;
    return;
  }

  if (strcmp(command, "MODE:OFFENSE") == 0)
  {
    kRobotMode = RobotMode::Offense;
    applyRobotModeSettings();
    return;
  }

  if (strcmp(command, "MODE:DEFENSE") == 0)
  {
    kRobotMode = RobotMode::Defense;
    applyRobotModeSettings();
    return;
  }

  if (strcmp(command, "CMD:CALIB_LINE_START") == 0)
  {
    lcdControl.lineCalibrationActive = true;
    movement.stop();
    return;
  }

  if (strcmp(command, "CMD:CALIB_LINE_STOP") == 0)
  {
    lcdControl.lineCalibrationActive = false;
    return;
  }

  handleStartPositionCommand(command);
}

void readLcdCommands()
{
  while (lcdSerial.available() > 0)
  {
    char incoming = (char)lcdSerial.read();
    if (incoming == '\r')
    {
      continue;
    }

    if (incoming == '\n')
    {
      lcdCommandBuffer[lcdCommandLength] = '\0';
      handleLcdCommand(lcdCommandBuffer);
      lcdCommandLength = 0;
      continue;
    }

    if (lcdCommandLength < (kLcdCommandBufferSize - 1))
    {
      lcdCommandBuffer[lcdCommandLength++] = incoming;
    }
  }
}

bool runRequestedCalibration()
{
  if (switches.calibration())
  {
    movement.stop();
    calibration.calibrateLineSensors();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    sendLcdCalibrationStatus();
    return true;
  }

  if (lcdControl.lineCalibrationActive)
  {
    movement.stop();
    calibration.calibrateLineSensors();
    sendLcdCalibrationStatus();
    return true;
  }

  return false;
}

void setup()
{
  applyRobotModeSettings();
  Serial.begin(9600);
  lcdSerial.begin(kLcdBaud);
  Serial2.begin(2000000);
  compassSensor.begin();
  compassSensor.callibrate();
}

double getHomeGoalAngle()
{
  if (isGoalBlueSelected())
  {
    return camera.yellowGoal;
  }
  return camera.blueGoal;
}

void runOffense()
{
  avoidanceAngle = -5;
  if (runRequestedCalibration())
  {
    movement.stop();
    calibration.calibrateLineSensors();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    return;
  }

  // Serial.println("Testing Line Sensors");
  lineDetection.Calculate();
  camera.CamCalc();
  lineAngle = lineDetection.getAngle();
  orbitAngle = orbit.CalculateRobotAngle(camera.ballAngle, camera.ballDist);
  if (isGoalBlueSelected())
  {
    Serial.println("blue goal");
    goalAngle = camera.blueGoal;
  }
  else
  {
    Serial.println("yellow goal");
    goalAngle = camera.yellowGoal;
  }

  if (goalAngle == -5)
  {
    goalAngle  = 0;
    aimingGoal = false;
  }
  else
  {
    aimingGoal = true;
  }

  Serial.println("Line Angle: "        + String(lineAngle));
  Serial.println("Robot Angle: "       + String(orbitAngle));
  Serial.println("Ball Angle: "        + String(camera.ballAngle));
  Serial.println("Goal Angle: "        + String(goalAngle));
  Serial.println("Ball dist: "         + String(camera.ballDist));
  Serial.println("Zeroed angle: "      + String(compassSensor.currentOffset()));
  Serial.println("Orientation angle: " + String(compassSensor.getOrientation()));

  movement.kickBackground();

  // Always drain Serial2 so no trajectory packets are silently dropped,
  // even when line avoidance overrides movement this iteration.
  trajectoryExecutor.processSerial();

  if (!switches.start())
  {
    movement.stop();
    return;
  }

  // â”€â”€ Line avoidance â€“ highest-priority safety override â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  // The trajectory executor is paused (not called) while avoiding the line.
  // It will resume seamlessly on the next iteration once clear.
  if (lineAngle != -5)
  {
    double avoidanceAngle = lineDetection.avoidanceAngle();
    Serial.println("Avoidance angle: " + String(avoidanceAngle));
    movement.movement(avoidanceAngle, lineAvoidanceSpeed, 0, false);
    return;
  }

  // â”€â”€ Kicker: fire when ball is secure and we are aligned with the goal â”€â”€â”€â”€â”€â”€â”€
  // THIS WILL HAVE TO CHANGE
  if (switches.lightgate() && fabs(goalAngle) < 5)
  {
    movement.kick();
    return;
  }

  // â”€â”€ Trajectory execution (Pipeline.md Steps 7â€“8) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  // execute() returns false when no active chunk has been received yet,
  // or after the grace period expires â€“ fall back to orbit-based approach.
  if   (!trajectoryExecutor.execute())
  {
    if (camera.ballAngle != -5)
    {
      movement.movement(orbitAngle, offenseSpeedFactor, goalAngle, aimingGoal);
    }
    else
    {
      movement.stop();
    }
  }
}

void runDefense()
{
  avoidanceAngle = -5;
  if (runRequestedCalibration())
  {
    return;
  }

  lineDetection.Calculate();
  lineAngle = lineDetection.getAngle();
  maxChordLength = lineDetection.getChordLengthFurthestPairNormalized();
  if (lineAngle != -5)
  {
    // Updates crossLine side memory based on angle wrap jumps.
    avoidanceAngle = lineDetection.avoidanceAngle();
    Serial.println("Avoidance Angle: " + String(avoidanceAngle));
  }
  bool crossLineState = lineDetection.getCrossLine();

  double homeGoalAngle = getHomeGoalAngle();
  movement.kickBackground();

  currentOffset = compassSensor.currentOffset();

  // Serial.println("Line Angle: " + String(lineAngle));
  // Serial.println("Ball Angle: " + String(camera.ballAngle));
  // Serial.println("Home Goal Angle: " + String(homeGoalAngle));
  // Serial.println("Max Normalized Activated Sensor Distance: " + String(maxChordLength));
  Serial.println("BALL DISTANCE: " + String(camera.ballDist));
  // Serial.println("Cross Line: " + String(crossLineState ? "true" : "false"));
  // Serial.println("Current offset: " + String(currentOffset));

  sendLcdTelemetry();

  if (!isStartEnabled())
  {
    movement.stop();
    return;
  }

  if (camera.ballAngle == -5)
  {
    movement.stop();
    return;
  }

  if (homeGoalAngle == -5)
  {
    movement.movement(camera.ballAngle, defenseSpeedFactor, 0, false);
    return;
  }
}


void loop()
{
  readLcdCommands();
  if (kRobotMode == RobotMode::Offense)
  {
    runOffense();
  }
  else
  {
    runDefense();
  }

  // for (int i = 0; i < 10; i++) {
  //   Serial.println();
  // }

  // delay(200);

}
