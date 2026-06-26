#include <Arduino.h>
#include <RobotContext.h>
#include <OffenseController.h>

void setup()
{
  initializeRobotContext();
  Serial.begin(9600);
  Serial.println("Testing Run");
  Serial3.begin(2000000);
  compassSensor.begin();
  modeControl->begin(115200);
  modeControl->applyRobotModeSettings();
}

void loop()
{
  linePCBComm.update();
  linePCBComm.setRobotHeadingDegrees(compassSensor.currentOffset());
  modeControl->readCommands();

  if (kRobotMode == RobotMode::Offense)
  {
    runOffense();
  }
  else
  {
    // runDefense();
  }
  modeControl->sendTelemetry(lineAngle, avoidanceAngle);
}
