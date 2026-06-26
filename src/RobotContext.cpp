#include <Arduino.h>

#include <RobotConfig.h>
#include <RobotContext.h>

RobotMode kRobotMode = defaultRobotMode;

CompassSensor compassSensor;
Switch switches;
Calibration calibration(compassSensor);
Motor* FL = nullptr;
Motor* FR = nullptr;
Motor* BL = nullptr;
Motor* BR = nullptr;
Movement* movement = nullptr;
Orbit orbit(1);
Cam camera;
LinePCBComm linePCBComm(Serial2);
ModeControl* modeControl = nullptr;

double lineAngle;
double currentOffset;
double orbitAngle;
double maxChordLength;
double goalAngle;
double avoidanceAngle;
bool aimingGoal;

static void initializeDriveMotors()
{
  pinMode(selectionPin, INPUT);
  const bool useDefaultMotorLayout = digitalRead(selectionPin) == HIGH;

  if (useDefaultMotorLayout)
  {
    FL = new Motor(pincontrolFLA, pincontrolFLB, pinspeedFL);
    FR = new Motor(pincontrolFRA, pincontrolFRB, pinspeedFR);
    BL = new Motor(pincontrolRLA, pincontrolRLB, pinspeedRL);
    BR = new Motor(pincontrolRRA, pincontrolRRB, pinspeedRR);
  }
  else
  {
    BR = new Motor(pincontrolFLA, pincontrolFLB, pinspeedFL);
    FR = new Motor(pincontrolFRA, pincontrolFRB, pinspeedFR);
    FL = new Motor(pincontrolRLB, pincontrolRLA, pinspeedRL);
    BL = new Motor(pincontrolRRB, pincontrolRRA, pinspeedRR);
  }
}

void initializeRobotContext()
{
  initializeDriveMotors();
  movement = new Movement(*FL, *FR, *BL, *BR, compassSensor);
  modeControl = new ModeControl(Serial8, linePCBComm, compassSensor, *movement, kRobotMode);
}
