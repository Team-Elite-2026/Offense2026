#include <Arduino.h>
#include <RobotContext.h>
#include <OffenseController.h>

void setup()
{
  initializeRobotContext();
  Serial.begin(9600);
  Serial.println("Testing Run");
  Serial3.begin(2000000);
<<<<<<< Updated upstream
  compassSensor.begin();
  // compassSensor.callibrate();
  modeControl.begin(115200);
  modeControl.applyRobotModeSettings();

=======
  modeControl->begin(115200);
  modeControl->sendBootMarker();
  compassSensor.begin();
  compassSensor.callibrate(modeControl->statusOutput());
  modeControl->applyRobotModeSettings();
>>>>>>> Stashed changes
}

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
  if (modeControl.state.lineCalibrationActive)
  {
    movement.stop();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
  }
  else
  {
    // Serial.println("Testing Line Sensors");
    linePCBComm.update();
    camera.CamCalc();
    lineAngle = linePCBComm.getLineAngle();
    orbitAngle = orbit.CalculateRobotAngle(camera.ballAngle, camera.ballDist);
    if (modeControl.state.goalIsBlue)
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

    // Serial.println("Offset: " + String(compassSensor.currentOffset()));
    Serial.println("Line Angle: " + String(lineAngle));
    Serial.println("Robot Angle: " + String(orbitAngle));
    Serial.println("Ball Angle: " + String(camera.ballAngle));
    Serial.println("Goal Angle: " + String(goalAngle));
    Serial.println("Ball dist:" + String(camera.ballDist));
    Serial.println("Robot Heading: " + String(compassSensor.currentOffset()));
    // Serial.println("Orientation angle" + String(compassSensor.getOrientation()));
    movement.kickBackground();
    if (lineAngle == -5)
    {
      if (modeControl.isStartEnabled())
      {
        if (modeControl.doWeHaveBall())
        {
          // movement.movement(0, 0.2, goalDesiredFieldAngle, aimingGoal); 
          if (fabs(goalAngle) < 5)
          { // if close to goal angle, kick
            movement.kick(); // wanna kick the ball to the goal
          }
        }
        else if (camera.ballAngle != -5)
        {
          movement.movement(orbitAngle, offenseSpeedFactor, 0, false); // wanna try to face dir of ball to get into dribbler so no trying to aim to the goal
          // movement.movement(orbitAngle, offenseSpeedFactor, goalAngle, aimingGoal); // j using default orbit aiming towards the goal if seen
        }
        else
        {
          movement.stop();
        }
      }
      else
      {
        movement.stop();
      }
    }
    else
    {
      double avoidanceAngle = linePCBComm.getAvoidanceAngle();
      Serial.println("Avoidance angle: " + String(avoidanceAngle));
      if (switches.start())
      {
        movement.movement(avoidanceAngle, lineAvoidanceSpeed, 0 , false); // Not turning while avoiding line can cause extra rotation when goal scoring meaning we still want to correct when we're goal scoring
      }
      else
      {
        movement.stop();
      }
    }
  }
}

// void runDefense()
// {
//   if (switches.calibration())
//   {
//     movement.stop();
//     calibration.calibrateLineSensors();
//     calibration.calibrateCompassSensor();
//     Serial.println("Calibrating");
//     return;
//   }

//   lineDetection.Calculate();
//   camera.CamCalc();
//   lineAngle = lineDetection.getAngle();
//   maxChordLength = lineDetection.getChordLengthFurthestPairNormalized();
//   if (lineAngle != -5)
//   {
//     // Updates crossLine side memory based on angle wrap jumps.
//     avoidanceAngle = lineDetection.avoidanceAngle();
//     Serial.println("Avoidance Angle: " + String(avoidanceAngle));
//   }
//   bool crossLineState = lineDetection.getCrossLine();

//   double homeGoalAngle = getHomeGoalAngle();
//   movement.kickBackground();

//   currentOffset = compassSensor.currentOffset();

//   // Serial.println("Line Angle: " + String(lineAngle));
//   // Serial.println("Ball Angle: " + String(camera.ballAngle));
//   // Serial.println("Home Goal Angle: " + String(homeGoalAngle));
//   // Serial.println("Max Normalized Activated Sensor Distance: " + String(maxChordLength));
//   Serial.println("BALL DISTANCE: " + String(camera.ballDist));
//   // Serial.println("Cross Line: " + String(crossLineState ? "true" : "false"));
//   // Serial.println("Current offset: " + String(currentOffset));

//   if (!switches.start())
//   {
//     movement.stop();
//     return;
//   }

//   if (camera.ballAngle == -5)
//   {
//     movement.stop();
//     return;
//   }

//   if (homeGoalAngle == -5)
//   {
//     movement.movement(camera.ballAngle, defenseSpeedFactor, 0, false);
//     return;
//   }
// }


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
