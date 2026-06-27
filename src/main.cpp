#include <Arduino.h>
#include <RobotContext.h>
#include <OffenseController.h>

void setup()
{
  initializeRobotContext();
  Serial.begin(9600);
  Serial.println("Testing Run");
  Serial3.begin(2000000);
  modeControl->begin(115200);
  modeControl->sendBootMarker();
  compassSensor.begin();
  compassSensor.callibrate(modeControl->statusOutput());
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
