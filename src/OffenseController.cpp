#include <Arduino.h>
#include <math.h>

#include <RobotConfig.h>
#include <RobotContext.h>
#include <OffenseController.h>

double getHomeGoalAngle()
{
  if (switches.goalSide())
  {
    return camera.yellowGoal;
  }
  return camera.blueGoal;
}

void runOffense()
{
  if (modeControl->state.lineCalibrationActive)
  {
    movement->stop();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
  }
  else
  {
    linePCBComm.update();
    camera.CamCalc();
    lineAngle = linePCBComm.getLineAngle();
    orbitAngle = orbit.CalculateRobotAngle(camera.ballAngle, camera.ballDist);
    if (modeControl->state.goalIsBlue)
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
      goalAngle = 0;
      aimingGoal = false;
    }
    else
    {
      aimingGoal = true;
    }

    Serial.println("Line Angle: " + String(lineAngle));
    Serial.println("Robot Angle: " + String(orbitAngle));
    Serial.println("Ball Angle: " + String(camera.ballAngle));
    Serial.println("Goal Angle: " + String(goalAngle));
    Serial.println("Ball dist:" + String(camera.ballDist));
    Serial.println("Robot Heading: " + String(compassSensor.currentOffset()));

    movement->kickBackground();
    if (lineAngle == -5)
    {
      if (modeControl->isStartEnabled())
      {
        if (modeControl->doWeHaveBall())
        {
          if (fabs(goalAngle) < 5)
          {
            movement->kick();
          }
        }
        else if (camera.ballAngle != -5)
        {
          movement->movement(orbitAngle, offenseSpeedFactor, 0, false);
        }
        else
        {
          movement->stop();
        }
      }
      else
      {
        movement->stop();
      }
    }
    else
    {
      double currentAvoidanceAngle = linePCBComm.getAvoidanceAngle();
      avoidanceAngle = currentAvoidanceAngle;
      Serial.println("Avoidance angle: " + String(currentAvoidanceAngle));
      if (switches.start())
      {
        movement->movement(currentAvoidanceAngle, lineAvoidanceSpeed, 0, false);
      }
      else
      {
        movement->stop();
      }
    }
  }
}
