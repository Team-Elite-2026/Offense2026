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

enum class RobotMode
{
  Offense,
  Defense
};

RobotMode kRobotMode = RobotMode::Defense;
double defenseSpeedFactor = 0.3;
double offenseSpeedFactor = 0.2;



double lineAngle, currentOffset, orbitAngle, maxChordLength, goalAngle, avoidanceAngle;
bool aimingGoal;

void setup()
{
  Serial.begin(9600);
  Serial2.begin(2000000);
  compassSensor.begin();
  compassSensor.callibrate();
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
  if (switches.calibration())
  {
    calibration.calibrateLineSensors();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
  }
  else
  {
    // Serial.println("Testing Line Sensors");
    lineDetection.Calculate();
    camera.CamCalc();
    lineAngle = lineDetection.getAngle();
    orbitAngle = orbit.CalculateRobotAngle(camera.ballAngle, camera.ballDist);
    if (switches.goalSide())
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
    movement.kickBackground();
    if (lineAngle == -5)
    {
      if (switches.start())
      {
        if (switches.lightgate())
        {
          // movement.movement(0, 0.2, goalDesiredFieldAngle, aimingGoal); 
          movement.rotateToGoal(-goalAngle, 0.2);
          if (fabs(goalAngle) < 5)
          { // if close to goal angle, kick
            movement.kick(); // wanna kick the ball to the goal
          }
        }
        else if (camera.ballAngle != -5)
        {
          // movement.movement(ballAngle, offenseSpeedFactor, camera.ballAngle, false); // wanna try to face dir of ball to get into dribbler so no trying to aim to the goal
          movement.movement(orbitAngle, offenseSpeedFactor, goalAngle, aimingGoal); // j using default orbit aiming towards the goal if seen
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
      double avoidanceAngle = lineDetection.avoidanceAngle();
      Serial.println("Avoidance angle: " + String(avoidanceAngle));
      if (switches.start())
      {
        movement.movement(avoidanceAngle, offenseSpeedFactor, 0, false); // Not turning while avoiding line can cause extra rotation when goal scoring meaning we still want to correct when we're goal scoring
      }
      else
      {
        movement.stop();
      }
    }
  }
}

void runDefense()
{
  if (switches.calibration())
  {
    calibration.calibrateLineSensors();
    calibration.calibrateCompassSensor();
    Serial.println("Calibrating");
    return;
  }

  lineDetection.Calculate();
  camera.CamCalc();
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

  Serial.println("Line Angle: " + String(lineAngle));
  Serial.println("Ball Angle: " + String(camera.ballAngle));
  Serial.println("Home Goal Angle: " + String(homeGoalAngle));
  Serial.println("Max Normalized Activated Sensor Distance: " + String(maxChordLength));
  Serial.println("Cross Line: " + String(crossLineState ? "true" : "false"));
  Serial.println("Current offset: " + String(currentOffset));

  if (!switches.start())
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

  double defenseMoveAngle = defense.defenseCalc(
      camera.ballAngle,
      homeGoalAngle,
      currentOffset,
      lineAngle,
      maxChordLength,
      crossLineState);

  Serial.println("Defense Move angle: " + String(defenseMoveAngle));

  if (defenseMoveAngle < 0)
  {
    movement.stop();
    return;
  }

  double desiredPerpendicularHeading = 0.0;
  bool desiredHeadingInBadZone = false;
  if (lineAngle != -5)
  {
    const double badZoneHeadingLimit = 55.0;
    double relNormalA = Trig::wrapAngle(lineAngle);
    double relNormalB = Trig::wrapAngle(lineAngle + 180.0);
    double fieldNormalA = compassSensor.robotRelativeToField(relNormalA);
    double fieldNormalB = compassSensor.robotRelativeToField(relNormalB);
    bool normalAInBadZone = fabs(fieldNormalA) > badZoneHeadingLimit;
    bool normalBInBadZone = fabs(fieldNormalB) > badZoneHeadingLimit;

    if (normalAInBadZone != normalBInBadZone)
    {
      desiredPerpendicularHeading = normalAInBadZone ? fieldNormalB : fieldNormalA;
    }
    else
    {
      double chosenRelativeNormal = (fabs(relNormalA) <= fabs(relNormalB)) ? relNormalA : relNormalB;
      desiredPerpendicularHeading = compassSensor.robotRelativeToField(chosenRelativeNormal);
    }

    desiredHeadingInBadZone = fabs(desiredPerpendicularHeading) > badZoneHeadingLimit;
    Serial.println("Field Relative Desired Heading: " + String(desiredPerpendicularHeading));
  }

  if (desiredHeadingInBadZone) {
    Serial.println("YOU ARE APPROACHING A BAD ZONE");
    movement.stop();
    return;
  }

  movement.movement(defenseMoveAngle, defenseSpeedFactor, desiredPerpendicularHeading, false);

  Serial.println("Desired Heading: " + String(desiredPerpendicularHeading));
}

void loop()
{
  if (kRobotMode == RobotMode::Offense)
  {
    runOffense();
  }
  else
  {
    runDefense();
  }

  for (int i = 0; i < 10; i++) {
    Serial.println();
  }

}
